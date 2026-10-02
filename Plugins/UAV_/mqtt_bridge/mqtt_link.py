# mqtt_bridge/mqtt_link.py
"""paho-mqtt 2.x wrapper, MQTT 5 only. The broker-facing half of a bridge.

Standard MQTT 5 only -- no HiveMQ-proprietary features -- so the broker can be
swapped without touching this file (brief section 3).

Three things here are easy to get wrong and expensive to debug later, so they are
done from the first commit rather than deferred:

1. on_message must not block. It does one put_nowait and returns. A blocking
   write from paho's network thread stalls PINGREQ, the broker drops the
   connection on keepalive, paho reconnects, and the cycle repeats -- which
   presents as "the broker is flaky" rather than as a bug here.

2. Subscriptions are re-established inside on_connect, every time. A clean
   session (which the aircraft uses deliberately) forgets them on every
   reconnect, so subscribing once at startup silently stops working after the
   first network blip.

3. paho's outgoing queue is bounded. Its default max_queued_messages is 0,
   meaning unlimited, and a QoS >= 1 publish issued while disconnected is kept
   and RE-SENT on reconnect (verified in paho 2.1.0 client.py: the QoS 0 path
   returns MQTT_ERR_NO_CONN and discards, while the QoS >= 1 path stores the
   message and resets its state to mqtt_ms_publish). On the to_vehicle topic that
   is an unbounded backlog of stale commands, which is exactly the hazard brief
   section 4 is about.
"""

import logging
import threading
import time

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
from paho.mqtt.subscribeoptions import SubscribeOptions

logger = logging.getLogger(__name__)

# Carries the publisher's original MessageExpiryInterval alongside the value the
# broker decrements. The difference is how long the broker held the message, and
# computing it this way needs no clock synchronisation between aircraft and GCS.
USER_PROP_ORIGINAL_EXPIRY = "exp"


class MqttLink:
    """One MQTT connection: publishes one topic, subscribes to another.

    `on_frame(topic, payload, properties)` is called on paho's network thread and
    MUST return immediately.
    """

    def __init__(self, cfg, client_id, subscriptions, on_frame, on_connected=None):
        self.cfg = cfg
        self.client_id = client_id
        self._subs = list(subscriptions)        # [(topic, qos), ...]
        self._on_frame = on_frame
        self._on_connected = on_connected

        self._connected = threading.Event()
        self._started = False
        self._recent_disconnects = []     # monotonic timestamps
        self._warned_about_flapping = False

        # Counters. Plain ints: written by paho's thread or the publisher thread,
        # read only for logging, where a torn read does not matter.
        self.connects = 0
        self.disconnects = 0
        self.published = 0
        self.publish_refused_offline = 0
        self.publish_refused_queue_full = 0
        self.received = 0
        self.callback_errors = 0

        self.client = mqtt.Client(
            CallbackAPIVersion.VERSION2,
            client_id=client_id,     # unique: a duplicate makes the broker kick
            protocol=mqtt.MQTTv5,    # required for message expiry and properties
        )
        self.client.reconnect_delay_set(
            min_delay=cfg.RECONNECT_MIN_DELAY, max_delay=cfg.RECONNECT_MAX_DELAY)
        self.client.max_queued_messages_set(cfg.MAX_QUEUED_MESSAGES)
        self.client.max_inflight_messages_set(cfg.MAX_INFLIGHT_MESSAGES)

        if cfg.MQTT_USERNAME:
            self.client.username_pw_set(cfg.MQTT_USERNAME, cfg.MQTT_PASSWORD or None)

        self._connect_props = Properties(PacketTypes.CONNECT)
        self._connect_props.SessionExpiryInterval = cfg.SESSION_EXPIRY

        self.client.on_connect = self._cb_connect
        self.client.on_disconnect = self._cb_disconnect
        self.client.on_message = self._cb_message

    # ---- lifecycle -------------------------------------------------------

    def start(self):
        """Begin connecting. Never blocks and never raises on a down broker.

        connect_async means a broker that is not up yet is not a startup failure;
        paho retries with the backoff set above. That matters because the GCS
        bridge may well start before the broker on a cold boot.
        """
        if self._started:
            return
        self._started = True

        logger.info("MQTT connecting to %s:%d as %s (MQTT 5, keepalive %ds -> "
                    "ungraceful link loss detected in ~%ds)",
                    self.cfg.BROKER_HOST, self.cfg.BROKER_PORT, self.client_id,
                    self.cfg.KEEPALIVE, self.cfg.link_loss_detect_seconds)

        self.client.connect_async(
            self.cfg.BROKER_HOST,
            self.cfg.BROKER_PORT,
            keepalive=self.cfg.KEEPALIVE,
            # Sticky: paho stores _clean_start and reuses it for every automatic
            # reconnect, so this single call covers them all.
            clean_start=self.cfg.CLEAN_START,
            properties=self._connect_props,
        )
        self.client.loop_start()

    def stop(self):
        """Disconnect cleanly and stop the network thread."""
        try:
            self.client.disconnect()
        except Exception:
            logger.exception("MQTT disconnect raised")
        finally:
            try:
                self.client.loop_stop()
            except Exception:
                logger.exception("MQTT loop_stop raised")
        logger.info("MQTT stopped (%s)", self.client_id)

    @property
    def connected(self):
        return self._connected.is_set()

    def wait_connected(self, timeout):
        return self._connected.wait(timeout)

    # ---- callbacks (paho 2.x VERSION2 + MQTTv5 signatures) ---------------

    def _cb_connect(self, client, userdata, connect_flags, reason_code, properties):
        if reason_code.is_failure:
            # Wrong credentials or a topic-permission refusal land here. paho
            # keeps retrying, so this must be loud or it looks like a hang.
            logger.error("MQTT connection refused by broker: %s", reason_code)
            return

        self.connects += 1
        self._connected.set()
        logger.info("MQTT connected as %s (session_present=%s)",
                    self.client_id, connect_flags.session_present)

        # Every connect, not just the first: a clean session forgets these.
        for topic, qos in self._subs:
            # noLocal is a free second guarantee against a publish/subscribe loop
            # if anyone ever misconfigures both bridges onto the same topic. The
            # separate from_vehicle/to_vehicle topics are the primary guard.
            result, _ = client.subscribe(
                topic, options=SubscribeOptions(qos=qos, noLocal=True))
            if result != mqtt.MQTT_ERR_SUCCESS:
                logger.error("MQTT subscribe to %s failed: %s", topic, result)
            else:
                logger.info("MQTT subscribed to %s (QoS %d)", topic, qos)

        if self._on_connected:
            try:
                self._on_connected(self)
            except Exception:
                self.callback_errors += 1
                logger.exception("on_connected hook raised")

    def _cb_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        self._connected.clear()
        self.disconnects += 1
        logger.warning("MQTT disconnected (%s); paho will retry with backoff "
                       "%d-%ds", reason_code,
                       self.cfg.RECONNECT_MIN_DELAY, self.cfg.RECONNECT_MAX_DELAY)
        self._check_for_flapping()

    def _check_for_flapping(self):
        """Name the likely cause of repeated disconnects: a duplicate client id.

        MQTT requires the broker to disconnect an existing session when a second
        client connects with the same id. The kicked client reconnects, kicks the
        other, and the two trade places indefinitely. The visible symptom is
        repeated disconnects with no obvious trigger, which reads as an unreliable
        broker -- so this says the actual cause out loud, once.

        Overwhelmingly the practical cause is a second copy of this bridge left
        running from an earlier test.
        """
        now = time.monotonic()
        self._recent_disconnects = [t for t in self._recent_disconnects if now - t < 60.0]
        self._recent_disconnects.append(now)
        if len(self._recent_disconnects) >= 3 and not self._warned_about_flapping:
            self._warned_about_flapping = True
            logger.warning(
                "MQTT has disconnected %d times in the last minute as client id "
                "%r. The usual cause is ANOTHER PROCESS USING THE SAME CLIENT ID "
                "-- most often a second copy of this bridge left running -- "
                "because the broker must kick the older session, which then "
                "reconnects and kicks this one. Check for duplicate bridge "
                "processes before suspecting the broker or the network.",
                len(self._recent_disconnects), self.client_id)

    def _cb_message(self, client, userdata, message):
        self.received += 1
        try:
            self._on_frame(message.topic, message.payload, message.properties)
        except Exception:
            self.callback_errors += 1
            logger.exception("on_frame handler raised for %s", message.topic)

    # ---- publishing ------------------------------------------------------

    def publish_frame(self, topic, payload, qos, expiry_s=None):
        """Publish one raw frame. Returns True if paho accepted it.

        Refuses outright while disconnected instead of letting paho queue a
        QoS >= 1 message for replay after reconnect. Together with the aircraft's
        clean session this is what stops an old ARM or mode change arriving
        minutes late -- see brief section 4.
        """
        if not self._connected.is_set():
            self.publish_refused_offline += 1
            return False

        props = None
        if expiry_s:
            props = Properties(PacketTypes.PUBLISH)
            props.MessageExpiryInterval = int(expiry_s)
            # MQTT 5 requires the broker to decrement MessageExpiryInterval by
            # however long it held the message. Sending the original alongside
            # lets the receiver work out the broker dwell time by subtraction,
            # with no shared clock. Note this covers broker dwell only, not time
            # spent in a client's own out-queue or in TCP retransmission.
            props.UserProperty = [(USER_PROP_ORIGINAL_EXPIRY, str(int(expiry_s)))]

        info = self.client.publish(topic, payload, qos=qos, retain=False, properties=props)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            # MQTT_ERR_QUEUE_SIZE means max_queued_messages was hit: the link is
            # down or far too slow for the offered rate. Counted, not raised.
            self.publish_refused_queue_full += 1
            return False

        self.published += 1
        return True

    def counters(self):
        return {
            "mqtt_connects": self.connects,
            "mqtt_disconnects": self.disconnects,
            "mqtt_published": self.published,
            "mqtt_received": self.received,
            "mqtt_refused_offline": self.publish_refused_offline,
            "mqtt_refused_queue_full": self.publish_refused_queue_full,
            "mqtt_callback_errors": self.callback_errors,
        }


def original_expiry(properties):
    """The publisher's original expiry from the 'exp' user property, or None.

    Returns None when the broker does not forward user properties, which callers
    must treat as "age unknown" rather than "age zero".
    """
    user_props = getattr(properties, "UserProperty", None) if properties else None
    if not user_props:
        return None
    for key, value in user_props:
        if key == USER_PROP_ORIGINAL_EXPIRY:
            try:
                return int(value)
            except (TypeError, ValueError):
                return None
    return None


def remaining_expiry(properties):
    """The broker-decremented MessageExpiryInterval, or None if absent."""
    if not properties:
        return None
    value = getattr(properties, "MessageExpiryInterval", None)
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None

# mqtt_bridge/staleness.py
"""How old is this command, and is that too old? Pure: no I/O, no clock sync.

The hazard (brief section 4): a command published while the aircraft is offline
could be delivered minutes later, so an ARM or a mode change issued before a
dropout executes long after the operator expected it.

HOW AGE IS MEASURED WITHOUT A SHARED CLOCK
------------------------------------------
MQTT 5 requires a broker to DECREMENT MessageExpiryInterval by the number of
seconds it held a message before forwarding it. The publisher also sends its
original value in an 'exp' user property. The difference between the two is how
long the broker held the message:

    dwell = original_expiry - remaining_expiry

No clock synchronisation between aircraft and GCS is needed, which matters
because there is none available.

WHAT THIS DOES NOT MEASURE -- READ BEFORE TRUSTING IT
-----------------------------------------------------
Dwell at the broker is only one of three delays:

  1. broker dwell                                    <- measured here
  2. time in the publisher's own paho out-queue      <- NOT measured
  3. TCP retransmission and round-trip time          <- NOT measured

Measuring true end-to-end age needs a synchronised clock. So a command that
passes this check is "the broker did not sit on it", NOT "this command is
fresh". Items 2 and 3 are bounded by MAX_QUEUED_MESSAGES and by the link
itself, not by this module.

The layered defence this belongs to, in order of how much work each does:

  layer 1  aircraft connects clean_start=True, session expiry 0 -- the broker
           has no session to queue into, so commands published while it is
           offline are discarded at publish time. THE PRIMARY DEFENCE.
  layer 0  the publisher refuses to publish while its own link is down.
  layer 3  a short MessageExpiryInterval, so the broker drops what it held.
  layer 6  this module: reject on measured dwell.
"""

import logging

logger = logging.getLogger(__name__)


def age_from_expiry(original_expiry_s, remaining_expiry_s):
    """Seconds the broker held the message, or None if not determinable.

    None means "unknowable", which callers must not treat as "zero".
    """
    if original_expiry_s is None or remaining_expiry_s is None:
        return None
    try:
        age = int(original_expiry_s) - int(remaining_expiry_s)
    except (TypeError, ValueError):
        return None
    # A broker that forwards the property unchanged yields 0. Negative would
    # mean the remaining value exceeded the original, which is nonsense -- treat
    # it as unknown rather than as negative age.
    return age if age >= 0 else None


def is_stale(original_expiry_s, remaining_expiry_s, max_age_s, fail_open=True):
    """(stale, age, reason). `stale` True means the caller should drop it.

    fail_open governs the case where age cannot be determined at all:

      True  -- deliver it, and let the caller log. One broker that does not
               forward user properties then cannot silently disable the entire
               command path, which would present as an unexplained hang.
      False -- drop it. Nothing of unknown age ever reaches the aircraft, at the
               cost of that failure mode.

    This is a safety trade-off, not a technical detail. It is a config flag
    (TO_VEHICLE_FAIL_OPEN) and both bridges log which way it is set at startup,
    so the active posture is never ambiguous.
    """
    age = age_from_expiry(original_expiry_s, remaining_expiry_s)

    if age is None:
        if fail_open:
            return (False, None, "age unknown, delivered (fail-open)")
        return (True, None, "age unknown, dropped (fail-closed)")

    if max_age_s is not None and age > max_age_s:
        return (True, age, f"held {age}s at the broker, limit {max_age_s}s")

    return (False, age, f"held {age}s")


def describe_posture(max_age_s, fail_open, require_link_online):
    """One line for the startup log, so the active posture is on the record."""
    return (
        f"stale-command posture: reject commands held longer than {max_age_s}s "
        f"at the broker; age-unknown commands are "
        f"{'DELIVERED (fail-open)' if fail_open else 'DROPPED (fail-closed)'}; "
        f"commands while the aircraft is known offline are "
        f"{'SUPPRESSED' if require_link_online else 'still sent'}. "
        f"Note this measures broker dwell only, not queueing or network delay."
    )

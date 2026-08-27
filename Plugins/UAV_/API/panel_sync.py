# API/panel_sync.py
#
# Replaces json_api_updater.py's GuiStateExporter. That existed because
# the old PySide6 GUI was a SEPARATE PROCESS that could only see a JSON
# file on disk. api.py's new PanelState is imported directly by ui.py in
# the SAME process — there's no separate process to bridge to anymore,
# so there's no file to write. This reads from SystemState (same
# boundary rule as the old exporter: only ever reads self.state) and
# calls straight into api.py's functions instead.
#
# Two things this has to get right that a naive "copy every field
# across" wouldn't:
#
#   1. menu_select is a 2-bit BINARY index (0-3) on the wire, but
#      api.sync_from_register() already does the one-hot conversion
#      internally — this just needs to pass the raw value through.
#
#   2. Most PayloadCommand fields ARE already sustained toggle bits on
#      the wire (laser_cont_mode stays True until pressed again) and
#      map straight to api.set_active(field, value). But config.py's
#      "Record" binding is a Toggle built from TWO separate momentary
#      fields (start_record/stop_record) — PayloadCommand has no single
#      "is recording" boolean, just two edge-triggered pulses. That
#      needs actual edge-tracking to become one sustained state, not a
#      direct copy — see _sync_record_toggle below. start_record/
#      stop_record are excluded from the generic per-field loop for
#      exactly this reason; syncing them there too would fight with
#      this method over the same key ("start_record", per
#      state.py's _key() picking field_on for Toggle bindings).

import threading
import time
import logging
from dataclasses import fields as dataclass_fields

from . import api

logger = logging.getLogger(__name__)

POLL_INTERVAL_SEC = 0.05   # in-process calls are cheap; no need for the old 0.1s

# Fields handled by _sync_record_toggle instead of the generic loop.
_TOGGLE_HANDLED_FIELDS = frozenset({"start_record", "stop_record"})


class PanelStateSync:

    def __init__(self, state):
        self.state = state
        self._stop = threading.Event()
        self._thread = None

        self._last_payload_snapshot = {}
        self._last_menu = None
        self._recording = False   # local edge-tracked state for the Record toggle
        self._prev_start_record = False
        self._prev_stop_record = False

    def start(self):
        self._sync_once(force=True)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="PanelStateSync"
        )
        self._thread.start()
        logger.info("PanelStateSync: started.")

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("PanelStateSync: stopped.")

    def _run_loop(self):
        while not self._stop.is_set():
            self._sync_once(force=False)
            self._stop.wait(POLL_INTERVAL_SEC)

    def _sync_once(self, force: bool):
        self._sync_menu(force)
        self._sync_payload(force)

    # ── Menu selection ─────────────────────────────────────────────────────

    def _sync_menu(self, force: bool):
        menu_select = self.state.get_Current_Menu
        if force or menu_select != self._last_menu:
            api.sync_from_register(menu_select)
            self._last_menu = menu_select

    # ── Payload flags ──────────────────────────────────────────────────────

    def _sync_payload(self, force: bool):
        payload_cmd = self.state.get_latest_payload_command()
        if payload_cmd is None:
            return

        snapshot = {
            f.name: getattr(payload_cmd, f.name)
            for f in dataclass_fields(payload_cmd)
            if f.name not in _TOGGLE_HANDLED_FIELDS
        }

        if force or snapshot != self._last_payload_snapshot:
            for field_name, value in snapshot.items():
                if force or value != self._last_payload_snapshot.get(field_name):
                    api.set_active(field_name, value)
            self._last_payload_snapshot = snapshot

        self._sync_record_toggle(payload_cmd)

    def _sync_record_toggle(self, payload_cmd):
        """
        start_record/stop_record are momentary edge-triggered pulses on
        the wire, not a sustained state — this turns them into one
        sustained "is recording" flag, keyed on "start_record" to match
        state.py's _key() (which picks field_on for Toggle bindings).
        Rising edge on start_record -> recording = True. Rising edge on
        stop_record -> recording = False. Only acts on rising edges,
        same edge-detection principle translator.py's EDGE_TABLEs use.

        Tracked in dedicated attributes, not inside _last_payload_snapshot
        — that dict gets wholesale-replaced whenever any OTHER field
        changes (28 other fields, so often), which would silently reset
        this edge-tracking on unrelated updates if it lived there.
        """
        if payload_cmd.start_record and not self._prev_start_record:
            self._recording = True
            api.set_active("start_record", True)
        elif payload_cmd.stop_record and not self._prev_stop_record:
            self._recording = False
            api.set_active("start_record", False)

        self._prev_start_record = payload_cmd.start_record
        self._prev_stop_record = payload_cmd.stop_record
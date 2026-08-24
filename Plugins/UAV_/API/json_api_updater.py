# API/json_api_updater.py
#
# Reads live payload-button state from SystemState and keeps a JSON file
# in sync for an external GUI to read. Deliberately touches nothing but
# `state` — no serial, no mavlink, no direct hardware access. If the GUI
# ever needs more than what's currently exposed on PayloadCommand, that's
# a SystemState/wire-protocol change, not something to work around here.
#
# Update triggers (both, per the design decision this was built against):
#   - Immediate: a fast poll loop (POLL_INTERVAL_SEC) diffs the current
#     PayloadCommand against the last one written and writes right away
#     on any change.
#   - Heartbeat: even with zero changes, a write is forced at least every
#     HEARTBEAT_INTERVAL_SEC, so the GUI can distinguish "nothing changed"
#     from "the backend died" by watching the file's mtime.
#
# Writes are atomic (temp file + os.replace) so the GUI never reads a
# half-written file mid-update, regardless of how it's polling.

import json
import os
import shutil
import threading
import time
import logging
from dataclasses import fields as dataclass_fields

logger = logging.getLogger(__name__)

# Level guidance, matching logging_config.py's existing convention elsewhere
# in this codebase:
#   DEBUG    every poll tick's heartbeat write (frequent, low value on its own)
#   INFO     start/stop, and every write that was triggered by a real change —
#            includes exactly which binding(s) changed and old -> new value
#   WARNING  (not currently used here — nothing in this module has a
#            "blocked, proceeding anyway" case)
#   ERROR    write failures (disk full, permissions, etc.)

POLL_INTERVAL_SEC = 0.1        # how often we check for changes
HEARTBEAT_INTERVAL_SEC = 1.0   # force a write at least this often even if nothing changed


class GuiStateExporter:
    """
    Owns one JSON file on disk (the schema is a static template — labels,
    button names, firmware handler names, command names, everything
    except `active` and `menu_register`'s selected-menu value never
    changes at runtime). Each update cycle re-reads the template's
    structure from memory, overlays live `active` values from the latest
    PayloadCommand, and writes the result out atomically.

    Needs exactly one thing from SystemState:
        state.get_latest_payload_command() -> PayloadCommand | None

    `state.get_menu_register()` — which menu is physically selected on
    the unit — is NOT called here, because nothing on the wire currently
    carries that value; menu_register lives only on the STM32 and is
    never transmitted. The "selected menu" field in the template is left
    exactly as loaded (whatever the template file says) until that gap
    is closed on the firmware/protocol side. See the module docstring
    above.
    """

    def __init__(self, state, json_path: str, template_path: str = None):
        self.state = state
        self.json_path = json_path
        # Defaults to reading the initial structure from json_path itself
        # if no separate template is given — either way, this file's
        # structure (labels, commands, etc.) is loaded once and only the
        # dynamic fields get mutated on each update.
        self._template = self._load_template(template_path or json_path)
        self._stop = threading.Event()
        self._thread = None

        # (field_name_or_None, path_to_binding_dict) pairs, built once
        # from the template so every update cycle is just a fast lookup +
        # write, not a fresh walk of the JSON structure every time.
        self._bindings_index = self._build_bindings_index(self._template)
        logger.info(f"GuiStateExporter: indexed {len(self._bindings_index)} "
                    f"live bindings out of {sum(len(m.get('bindings', [])) for m in self._template.get('menus', []))} total")

        self._last_written_snapshot = None
        self._last_write_time = 0.0

    # ── Public lifecycle ──────────────────────────────────────────────────

    def start(self):
        """Writes an initial snapshot immediately, then starts the
        background poll/heartbeat thread."""
        payload_cmd = self.state.get_latest_payload_command()
        initial_snapshot = self._snapshot_from_payload(payload_cmd)
        self._log_diff(self._last_written_snapshot, initial_snapshot)
        self._write_snapshot(snapshot=initial_snapshot, force=True)

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="GuiStateExporter"
        )
        self._thread.start()
        logger.info(f"GuiStateExporter: started, writing to {self.json_path}")

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("GuiStateExporter: stopped.")

    # ── Background loop ───────────────────────────────────────────────────

    def _run_loop(self):
        while not self._stop.is_set():
            now = time.monotonic()
            payload_cmd = self.state.get_latest_payload_command()
            snapshot = self._snapshot_from_payload(payload_cmd)

            changed = snapshot != self._last_written_snapshot
            heartbeat_due = (now - self._last_write_time) >= HEARTBEAT_INTERVAL_SEC

            if changed:
                self._log_diff(self._last_written_snapshot, snapshot)
                self._write_snapshot(snapshot=snapshot, force=False)
            elif heartbeat_due:
                logger.debug(f"GuiStateExporter: heartbeat write, no change ({self.json_path})")
                self._write_snapshot(snapshot=snapshot, force=False)

            self._stop.wait(POLL_INTERVAL_SEC)

    def _log_diff(self, old_snapshot, new_snapshot):
        """Logs exactly which binding(s) flipped and old -> new, not just
        'something changed' — this is the actual point of adding logging
        here: being able to see in the log that e.g. ('laser', 0) went
        False -> True at a given timestamp, without needing to diff two
        JSON snapshots by hand."""
        if old_snapshot is None:
            logger.info(f"GuiStateExporter: first snapshot, {len(new_snapshot)} bindings")
            return
        changed_keys = [k for k in new_snapshot if new_snapshot[k] != old_snapshot.get(k)]
        for key in changed_keys:
            menu_id, idx = key
            logger.info(
                f"GuiStateExporter: {menu_id}[{idx}] "
                f"{old_snapshot.get(key)} -> {new_snapshot[key]}"
            )

    # ── Snapshot / diff ────────────────────────────────────────────────────

    def _snapshot_from_payload(self, payload_cmd):
        """
        Returns a plain dict of {binding_key: bool} for every binding
        that has a live field mapping — cheap to compare with `!=` on
        every poll tick, which is why this exists separately from
        actually mutating/writing the full template structure.
        """
        snapshot = {}
        for binding_key, field_spec in self._bindings_index.items():
            snapshot[binding_key] = self._resolve_active(payload_cmd, field_spec)
        return snapshot

    @staticmethod
    def _resolve_active(payload_cmd, field_spec):
        if payload_cmd is None:
            return False
        kind, names = field_spec
        if kind == "single":
            return bool(getattr(payload_cmd, names[0], False))
        if kind == "on_off":
            # e.g. "Record": active while either the start or stop bit is
            # the most recently meaningful one. Both being momentary
            # edge-style bits on the firmware side, showing "active" for
            # either is the closest single boolean can represent a
            # two-command toggle without inventing tri-state UI.
            return bool(getattr(payload_cmd, names[0], False)) or bool(getattr(payload_cmd, names[1], False))
        return False

    def _build_bindings_index(self, template):
        """
        Walks the template once at startup and records, for every binding
        that declares a `field` (or `field_on`/`field_off` pair), which
        PayloadCommand attribute(s) it maps to. Bindings with neither
        (the `"action_type": "unassigned"` placeholders) are skipped —
        there's nothing live to show for them yet.
        """
        index = {}
        for menu in template.get("menus", []):
            for i, binding in enumerate(menu.get("bindings", [])):
                key = (menu["id"], i)
                if "field" in binding:
                    index[key] = ("single", (binding["field"],))
                elif "field_on" in binding and "field_off" in binding:
                    index[key] = ("on_off", (binding["field_on"], binding["field_off"]))
        return index

    # ── Writing ────────────────────────────────────────────────────────────

    def _write_snapshot(self, snapshot=None, force=False):
        if snapshot is None:
            payload_cmd = self.state.get_latest_payload_command()
            snapshot = self._snapshot_from_payload(payload_cmd)

        doc = self._template  # structure only mutated below, never replaced wholesale
        for menu in doc.get("menus", []):
            for i, binding in enumerate(menu.get("bindings", [])):
                key = (menu["id"], i)
                if key in self._bindings_index:
                    binding["active"] = snapshot[key]

        self._atomic_write(doc)
        self._last_written_snapshot = snapshot
        self._last_write_time = time.monotonic()

    def _atomic_write(self, doc):
        """Write to a temp file in the same directory, then os.replace —
        this is what guarantees the GUI never sees a partially-written
        file, even if it happens to poll mid-write. A plain open()+write()
        to json_path directly would risk exactly that race."""
        tmp_path = self.json_path + ".tmp"
        try:
            with open(tmp_path, "w") as f:
                json.dump(doc, f, indent=2)
            os.replace(tmp_path, self.json_path)
        except OSError as e:
            logger.error(f"GuiStateExporter: write failed: {e}")

    @staticmethod
    def _load_template(path):
        with open(path, "r") as f:
            doc = json.load(f)
        logger.info(f"GuiStateExporter: loaded template from {path} "
                    f"({len(doc.get('menus', []))} menus)")
        return doc
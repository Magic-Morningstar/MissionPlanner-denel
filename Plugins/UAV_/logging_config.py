# logging_config.py
#
# Replaces system_Print with Python's standard logging module. Call
# setup_logging() ONCE, at the very top of main.py, before any other
# module logs anything.
#
# Why this instead of the old system_Print(msg):
#   - Levels. system_Print had one severity for everything — a "connected"
#     message and a "watchdog appears frozen" message looked identical.
#     Now you can show only WARNING+ on the console while still capturing
#     full DEBUG detail to a file, and change your mind at runtime without
#     touching call sites.
#   - Per-module names. Every logger is named after its file
#     (logging.getLogger(__name__)) — log lines now say WHERE they came
#     from, which matters a lot once you have a dozen threads.
#   - Thread name in the format string. This codebase is heavily
#     multithreaded (SerialReader, SerialProcessor, ManualControl,
#     TakeoffSequence, both MavlinkWorker loops...) — "which thread said
#     this" used to be a guessing game. Now it's just in the log line.
#   - Rotation. A file that grows forever is its own bug, eventually.
#   - Performance: see the note on lazy formatting below.

import logging
import logging.handlers
import sys

DEFAULT_LOG_FILE = "controller.log"

# ── Level guidance (document this for your own future call sites) ─────────
#   DEBUG    raw frames/packets, joystick & pot values, per-tick internals
#   INFO     commands received, state transitions, connect/disconnect
#   WARNING  guard rejections ("blocked — not armed"), cancelled operations
#   ERROR    connection failures, malformed frames, caught exceptions
#   CRITICAL watchdog timeout firing, emergency triggered


def setup_logging(file_level=logging.DEBUG, console_level=logging.INFO,
                   log_file=DEFAULT_LOG_FILE):
    root = logging.getLogger()
    root.setLevel(file_level)
    root.handlers.clear()

    file_fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d [%(threadName)-16s] %(levelname)-8s %(name)-28s %(message)s",
        datefmt="%H:%M:%S",
    )
    console_fmt = logging.Formatter(
        "%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    console = logging.StreamHandler(sys.stdout)
    console.setLevel(console_level)
    console.setFormatter(console_fmt)
    root.addHandler(console)

    file_handler = logging.handlers.RotatingFileHandler(
        log_file, maxBytes=5 * 1024 * 1024, backupCount=5
    )
    file_handler.setLevel(file_level)
    file_handler.setFormatter(file_fmt)
    root.addHandler(file_handler)

    return root

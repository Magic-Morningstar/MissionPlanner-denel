# utils/helper.py
#
# Your real project already has this file with more in it (print gating
# by config.DEBUG_MODE, etc). This is a minimal stand-in so the testing/
# harness runs standalone without needing the rest of your project.

import time


def system_Print(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}")

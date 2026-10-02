# mqtt_bridge/tests/test_no_heavy_imports.py
"""The bridge must stay independent of the rest of the UAV_ package.

The aircraft-side deployable is just mqtt_bridge/ plus logging_config.py and
utils/connection_manager.py. That host is headless, so a stray import of PySide6
-- or of the STM32 serial stack, or the Qt operator GUI -- would make the bridge
uninstallable there. An import creeps in easily and the consequence only shows up
on the aircraft, so it is pinned here.

Run in a subprocess because the pytest process has already imported plenty.
"""

import subprocess
import sys
from pathlib import Path

import pytest

FORBIDDEN = ("PySide6", "state", "serial_controller", "commands", "API", "Menu_UI")

UAV_ROOT = Path(__file__).resolve().parent.parent.parent

PROBE = """
import sys
import {module}
leaked = sorted({{
    name.split(".")[0] for name in sys.modules
    if name.split(".")[0] in {forbidden!r}
}})
print(",".join(leaked))
"""


@pytest.mark.parametrize("module", [
    "mqtt_bridge.framing",
    "mqtt_bridge.topics",
    "mqtt_bridge.bridge_config",
    "mqtt_bridge.mqtt_link",
    "mqtt_bridge.mavlink_endpoint",
    "mqtt_bridge.bridge_core",
    "mqtt_bridge.aircraft_bridge",
    "mqtt_bridge.gcs_bridge",
])
def test_module_pulls_in_nothing_heavy(module):
    result = subprocess.run(
        [sys.executable, "-c", PROBE.format(module=module, forbidden=FORBIDDEN)],
        cwd=str(UAV_ROOT), capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, f"importing {module} failed:\n{result.stderr}"
    leaked = result.stdout.strip()
    assert leaked == "", f"{module} pulled in {leaked}"


def test_pure_modules_import_without_paho_or_pymavlink():
    """framing, topics and bridge_config must not need the third-party deps.

    Keeps the unit-testable core testable on a machine with nothing installed,
    and keeps the dependency boundary where the plan put it.
    """
    probe = (
        "import sys\n"
        "for name in ('paho', 'pymavlink'):\n"
        "    sys.modules[name] = None\n"   # any import of these will now fail
        "import mqtt_bridge.framing, mqtt_bridge.topics, mqtt_bridge.bridge_config\n"
        "print('ok')\n"
    )
    result = subprocess.run([sys.executable, "-c", probe], cwd=str(UAV_ROOT),
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout

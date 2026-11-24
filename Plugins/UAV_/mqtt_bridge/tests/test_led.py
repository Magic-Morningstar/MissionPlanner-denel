# mqtt_bridge/tests/test_led.py
"""Status LED: state choice, Cube activity, sysfs handling, the blink loop."""

import threading

import pytest

from mqtt_bridge import led as L


def fake_sysfs(tmp_path, trigger="none [mmc0] timer heartbeat"):
    d = tmp_path / "ACT"
    d.mkdir()
    (d / "trigger").write_text(trigger)
    (d / "brightness").write_text("0")
    return d


@pytest.mark.parametrize("cube, broker, expected", [
    (True, True, L.OK),
    (False, True, L.NO_CUBE),
    (True, False, L.NO_BROKER),
    (False, False, L.NO_CUBE),      # Cube first: fix that at the bench
])
def test_choose_state(cube, broker, expected):
    assert L.choose_state(cube, broker) == expected


def test_every_pattern_lasts_two_seconds_and_differs():
    for steps in L.PATTERNS.values():
        assert sum(s for _, s in steps) == pytest.approx(2.0)
        assert any(on for on, _ in steps) and any(not on for on, _ in steps)
    assert len({tuple(p) for p in L.PATTERNS.values()}) == 3


def test_cube_activity():
    a = L.CubeActivity(quiet_s=3.0)
    assert a.update(100, 0.0) is False     # a counter alone proves nothing yet
    assert a.update(100, 1.0) is False
    assert a.update(150, 2.0) is True
    assert a.update(150, 4.9) is True
    assert a.update(150, 5.1) is False     # quiet for 3 s
    assert a.update(151, 6.0) is True


def test_sysfs_take_set_restore(tmp_path):
    d = fake_sysfs(tmp_path)
    led = L.SysfsLed(d)
    assert led.take()
    assert (d / "trigger").read_text() == "none"
    assert led.set(True) and (d / "brightness").read_text() == "1"
    assert led.set(False) and (d / "brightness").read_text() == "0"
    led.restore()
    assert (d / "trigger").read_text() == "mmc0"


def test_missing_led_disables_quietly(tmp_path):
    led = L.SysfsLed(tmp_path / "nope")
    assert led.take() is False
    assert led.set(True) is False
    led.restore()                           # nothing to restore, no error


class FakeLed:
    def __init__(self, stop, after):
        self.path, self.writes, self.stop, self.after = "fake", [], stop, after
        self.taken = self.restored = False

    def take(self):
        self.taken = True
        return True

    def set(self, on):
        self.writes.append(on)
        if len(self.writes) >= self.after:
            self.stop.set()
        return True

    def restore(self):
        self.restored = True


def tiny(patterns):
    return {k: [(on, 0.001) for on, _ in v] for k, v in patterns.items()}


def test_loop_shows_no_cube_without_frames_and_restores():
    stop = threading.Event()
    fake = FakeLed(stop, after=4)
    L.run_status_led(fake, stop, frame_count=lambda: 0, broker_up=lambda: True,
                     patterns=tiny(L.PATTERNS))
    assert fake.taken and fake.restored
    assert fake.writes == [on for on, _ in L.PATTERNS[L.NO_CUBE]]


def test_loop_shows_ok_when_frames_flow():
    stop = threading.Event()
    fake = FakeLed(stop, after=6)
    count = iter(range(0, 10_000, 10))
    t = iter(range(10_000))
    L.run_status_led(fake, stop, frame_count=lambda: next(count),
                     broker_up=lambda: True, patterns=tiny(L.PATTERNS),
                     clock=lambda: float(next(t)) * 0.1)
    # First cycle has seen nothing yet, then frames are flowing.
    assert fake.writes[-2:] == [True, False]
    assert fake.writes[4:6] == [on for on, _ in L.PATTERNS[L.OK]]


def test_loop_stops_if_led_write_fails(tmp_path):
    stop = threading.Event()
    led = L.SysfsLed(fake_sysfs(tmp_path))
    led.take = lambda: True
    led.set = lambda on: False
    L.run_status_led(led, stop, frame_count=lambda: 0, broker_up=lambda: True)
    assert not stop.is_set()               # returned on its own, quickly


def test_loop_survives_a_broken_state_callback():
    stop = threading.Event()
    fake = FakeLed(stop, after=4)

    def boom():
        raise RuntimeError("x")

    L.run_status_led(fake, stop, frame_count=boom, broker_up=lambda: True,
                     patterns=tiny(L.PATTERNS))
    assert fake.writes == [on for on, _ in L.PATTERNS[L.NO_CUBE]]

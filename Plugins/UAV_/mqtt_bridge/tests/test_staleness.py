# mqtt_bridge/tests/test_staleness.py
"""Stale-command age measurement and the fail-open/fail-closed decision."""

import pytest

from mqtt_bridge.staleness import age_from_expiry, describe_posture, is_stale


# --------------------------------------------------------------------------
# Age, derived from the broker decrementing MessageExpiryInterval
# --------------------------------------------------------------------------

@pytest.mark.parametrize("original,remaining,expected", [
    (30, 30, 0),     # forwarded immediately
    (30, 25, 5),     # held 5s
    (30, 1, 29),     # held almost to expiry
    (3, 3, 0),
])
def test_age_from_dwell(original, remaining, expected):
    assert age_from_expiry(original, remaining) == expected


@pytest.mark.parametrize("original,remaining", [
    (None, 30),      # publisher sent no 'exp' property
    (30, None),      # broker did not forward MessageExpiryInterval
    (None, None),
    ("x", 30),       # unparseable
    (30, 40),        # remaining > original is nonsense, not negative age
])
def test_age_unknown_is_none_not_zero(original, remaining):
    """None must never be confused with 0 -- 'unknown' is not 'fresh'."""
    assert age_from_expiry(original, remaining) is None


# --------------------------------------------------------------------------
# The staleness decision
# --------------------------------------------------------------------------

def test_fresh_command_passes():
    stale, age, _ = is_stale(30, 28, max_age_s=30)
    assert stale is False and age == 2


def test_command_held_too_long_is_dropped():
    stale, age, reason = is_stale(60, 10, max_age_s=30)
    assert stale is True and age == 50
    assert "limit 30s" in reason


def test_boundary_is_not_stale():
    """Exactly at the limit is allowed; one second past is not."""
    assert is_stale(60, 30, max_age_s=30)[0] is False   # age 30
    assert is_stale(60, 29, max_age_s=30)[0] is True    # age 31


def test_fail_open_delivers_unknown_age():
    """The configured posture: a broker quirk must not kill the command path."""
    stale, age, reason = is_stale(None, None, max_age_s=30, fail_open=True)
    assert stale is False
    assert age is None
    assert "fail-open" in reason


def test_fail_closed_drops_unknown_age():
    """The other posture, implemented and tested so switching is a config change."""
    stale, age, reason = is_stale(None, None, max_age_s=30, fail_open=False)
    assert stale is True
    assert age is None
    assert "fail-closed" in reason


def test_unknown_age_ignores_max_age_either_way():
    """With no age there is nothing to compare, whatever the limit is."""
    assert is_stale(None, None, max_age_s=0, fail_open=True)[0] is False
    assert is_stale(None, None, max_age_s=9999, fail_open=False)[0] is True


# --------------------------------------------------------------------------
# The posture must be legible in the log
# --------------------------------------------------------------------------

def test_posture_line_states_the_actual_configuration():
    line = describe_posture(30, fail_open=True, require_link_online=False)
    assert "30s" in line
    assert "fail-open" in line.lower()
    assert "still sent" in line
    # The caveat must travel with it: this measures broker dwell only.
    assert "broker dwell" in line


def test_posture_line_reflects_fail_closed():
    line = describe_posture(5, fail_open=False, require_link_online=True)
    assert "fail-closed" in line.lower()
    assert "SUPPRESSED" in line

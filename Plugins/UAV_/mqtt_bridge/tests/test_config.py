# mqtt_bridge/tests/test_config.py

import logging
import os

import pytest

from mqtt_bridge.bridge_config import (
    CONTESTED_PORTS, DEFAULTS, ENV_PREFIX, build_config, coerce, load_config,
    parse_secrets_file, resolve_log_path, warn_about_contested_port,
)


# --------------------------------------------------------------------------
# Precedence: defaults < secrets < env < cli
# --------------------------------------------------------------------------

def test_defaults_only():
    cfg = build_config()
    assert cfg.VEHICLE_ID == "uav01"
    assert cfg.BROKER_PORT == 1883
    assert cfg.QOS_TO_VEHICLE == 1


def test_full_precedence_chain():
    cfg = build_config(
        secrets={"BROKER_HOST": "from-secrets", "MQTT_PASSWORD": "pw"},
        env={ENV_PREFIX + "BROKER_HOST": "from-env", ENV_PREFIX + "VEHICLE_ID": "env-vid"},
        cli={"BROKER_HOST": "from-cli"},
    )
    assert cfg.BROKER_HOST == "from-cli"      # cli beats env beats secrets
    assert cfg.VEHICLE_ID == "env-vid"        # env beats default
    assert cfg.MQTT_PASSWORD == "pw"          # secrets beats default


def test_unset_cli_flag_is_not_an_override():
    """argparse gives None for a flag the user did not pass."""
    cfg = build_config(env={ENV_PREFIX + "BROKER_HOST": "from-env"},
                       cli={"BROKER_HOST": None})
    assert cfg.BROKER_HOST == "from-env"


def test_env_requires_the_prefix():
    cfg = build_config(env={"BROKER_HOST": "no-prefix"})
    assert cfg.BROKER_HOST == DEFAULTS["BROKER_HOST"]


def test_env_key_is_case_insensitive_after_the_prefix():
    cfg = build_config(env={ENV_PREFIX + "broker_host": "lower"})
    assert cfg.BROKER_HOST == "lower"


def test_unknown_keys_are_ignored_not_fatal():
    cfg = build_config(env={ENV_PREFIX + "NOT_A_REAL_KEY": "x"},
                       cli={"ALSO_NOT_REAL": "y"})
    assert cfg.VEHICLE_ID == "uav01"


# --------------------------------------------------------------------------
# Coercion, and the fall-back-with-an-ERROR contract
# --------------------------------------------------------------------------

@pytest.mark.parametrize("raw,expected", [
    ("1", True), ("true", True), ("TRUE", True), ("yes", True), ("on", True),
    ("0", False), ("false", False), ("no", False), ("off", False),
])
def test_bool_coercion(raw, expected):
    assert coerce("CLEAN_START", raw, True) is expected


def test_int_coercion():
    assert coerce("BROKER_PORT", "8883", 1883) == 8883
    assert coerce("BROKER_PORT", 8883, 1883) == 8883


def test_bad_value_falls_back_and_logs_an_error(caplog):
    """A typo in one variable must not stop an aircraft bridge from starting."""
    with caplog.at_level(logging.ERROR):
        cfg = build_config(env={ENV_PREFIX + "BROKER_PORT": "188e"})
    assert cfg.BROKER_PORT == 1883                      # fell back
    assert "BROKER_PORT" in caplog.text                 # and said so, by name


def test_bad_bool_falls_back_and_logs(caplog):
    with caplog.at_level(logging.ERROR):
        cfg = build_config(env={ENV_PREFIX + "CLEAN_START": "maybe"})
    assert cfg.CLEAN_START is True
    assert "CLEAN_START" in caplog.text


# --------------------------------------------------------------------------
# Safety-relevant defaults, pinned so a casual change is visible in review
# --------------------------------------------------------------------------

def test_telemetry_qos_is_zero_for_safety_not_only_bandwidth():
    """paho DROPS QoS 0 when offline but RE-SENDS QoS >= 1 after reconnect.

    So QOS_FROM_VEHICLE=0 is what prevents a burst of stale telemetry over LTE on
    every reconnect. Raising it needs a deliberate decision, not a default edit.
    """
    assert DEFAULTS["QOS_FROM_VEHICLE"] == 0
    assert DEFAULTS["QOS_TO_VEHICLE"] == 1


def test_paho_out_queue_is_bounded():
    """paho's own default is 0 = unlimited; on to_vehicle that is a backlog."""
    assert DEFAULTS["MAX_QUEUED_MESSAGES"] > 0
    assert DEFAULTS["MAX_INFLIGHT_MESSAGES"] > 0


def test_clean_start_is_the_primary_stale_command_defence():
    assert DEFAULTS["CLEAN_START"] is True
    assert DEFAULTS["SESSION_EXPIRY"] == 0


def test_gcs_bind_is_loopback_and_outside_the_contested_range():
    host, port = build_config().gcs_bind_address()
    assert host == "127.0.0.1"          # never 0.0.0.0 on a ground station
    assert port == 14570
    assert port not in CONTESTED_PORTS


def test_link_loss_detection_time_is_derived_and_reported():
    cfg = build_config(cli={"KEEPALIVE": 20})
    assert cfg.link_loss_detect_seconds == 30
    assert build_config(cli={"KEEPALIVE": 4}).link_loss_detect_seconds == 6


# --------------------------------------------------------------------------
# Contested ports
# --------------------------------------------------------------------------

@pytest.mark.parametrize("port", [14550, 14551, 14555, 14559])
def test_contested_ports_warn(port, caplog):
    with caplog.at_level(logging.WARNING):
        assert warn_about_contested_port(port) is True
    assert "SO_REUSEADDR" in caplog.text


@pytest.mark.parametrize("port", [14570, 14549, 14560, 5760])
def test_clean_ports_do_not_warn(port):
    assert warn_about_contested_port(port) is False


# --------------------------------------------------------------------------
# Secrets
# --------------------------------------------------------------------------

def test_parse_secrets_file():
    parsed = parse_secrets_file(
        "# a comment\n"
        "\n"
        "MQTT_USERNAME=aircraft\n"
        'MQTT_PASSWORD="has spaces and # hash"\n'
        "  BROKER_HOST = spaced  \n"
        "not_a_pair\n"
    )
    assert parsed["MQTT_USERNAME"] == "aircraft"
    assert parsed["MQTT_PASSWORD"] == "has spaces and # hash"
    assert parsed["BROKER_HOST"] == "spaced"
    assert "NOT_A_PAIR" not in parsed


def test_password_is_not_leaked_by_loggable_or_repr():
    cfg = build_config(secrets={"MQTT_PASSWORD": "hunter2"})
    assert cfg.loggable()["MQTT_PASSWORD"] == "<set>"
    assert "hunter2" not in repr(cfg)
    assert "hunter2" not in str(cfg.loggable())


def test_loggable_covers_every_field():
    cfg = build_config()
    assert set(cfg.loggable()) == set(DEFAULTS)


# --------------------------------------------------------------------------
# Log path: absolute and CWD-independent
# --------------------------------------------------------------------------

def test_resolve_log_path_is_absolute_and_cwd_independent(tmp_path, monkeypatch):
    """The bug this guards against: a relative path resolving into system32."""
    monkeypatch.chdir(tmp_path)
    first = resolve_log_path("x.log", log_dir=str(tmp_path / "logs"))
    assert os.path.isabs(first)

    other = tmp_path / "elsewhere"
    other.mkdir()
    monkeypatch.chdir(other)
    assert resolve_log_path("x.log", log_dir=str(tmp_path / "logs")) == first


def test_resolve_log_path_creates_the_directory(tmp_path):
    target = tmp_path / "deep" / "nested"
    path = resolve_log_path("bridge.log", log_dir=str(target))
    assert target.is_dir()
    assert path.endswith("bridge.log")


def test_resolve_log_path_env_override(tmp_path, monkeypatch):
    monkeypatch.setenv(ENV_PREFIX + "LOG_DIR", str(tmp_path / "fromenv"))
    assert str(tmp_path / "fromenv") in resolve_log_path("a.log")


@pytest.mark.skipif(os.name != "nt", reason="Windows ProgramData convention")
def test_resolve_log_path_windows_uses_programdata(monkeypatch):
    """Matches where DenelPythonLauncher already writes denel_python.log."""
    monkeypatch.delenv(ENV_PREFIX + "LOG_DIR", raising=False)
    assert "Denel GCS" in resolve_log_path("bridge.log")


def test_load_config_reads_no_secrets_file_when_absent(tmp_path):
    cfg = load_config(directory=tmp_path, env={})
    assert cfg.MQTT_PASSWORD == ""


def test_load_config_reads_a_secrets_file(tmp_path):
    (tmp_path / "secrets.env").write_text("MQTT_PASSWORD=fromfile\n", encoding="utf-8")
    cfg = load_config(directory=tmp_path, env={})
    assert cfg.MQTT_PASSWORD == "fromfile"


def test_stdin_shutdown_is_off_by_default():
    """Auto-arming it broke launches outright, so the default must stay off.

    "stdin is not a terminal" is equally true for a held-open pipe and for stdin
    that is closed or /dev/null. systemd supplies /dev/null, so an auto-armed
    watcher reads EOF immediately and the bridge exits the instant it starts --
    observed in testing, with "launcher closed stdin" as the only clue.
    """
    assert DEFAULTS["STDIN_SHUTDOWN"] is False
    assert build_config().STDIN_SHUTDOWN is False
    assert build_config(cli={"STDIN_SHUTDOWN": True}).STDIN_SHUTDOWN is True


def test_to_vehicle_expiry_is_the_value_the_team_chose():
    """30 s, raised from 3 s after Phase 1c. Pinned so it cannot drift back.

    3 s intermittently discarded legitimate commands on a stalling link, with
    silence as the only symptom. 30 s is an interim bench/simulator setting, NOT
    a flight setting: until the receive-side dwell check exists, this value is
    the only bound on how stale a command can be when it reaches the aircraft.
    Change it together with TO_VEHICLE_FAIL_OPEN, not on its own.
    """
    assert DEFAULTS["TO_VEHICLE_EXPIRY"] == 30
    # Still overridable, which is how the A/B comparison was run.
    assert build_config(env={ENV_PREFIX + "TO_VEHICLE_EXPIRY": "3"}).TO_VEHICLE_EXPIRY == 3

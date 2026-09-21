# state/system_config.py

SERIAL_PORT = "COM7"          # unused — find_stm_port() scans by VID:PID
BAUDRATE = 115200
SERIAL_TIMEOUT = 1
CONNECTION_CHECK_TIMEOUT = 3

MAVLINK_STATE_PORT = 'udpin:0.0.0.0:14550'

MAVLINK_COMMAND_PORT = 'udpin:0.0.0.0:14550'
HEARTBEAT_TIMEOUT = 5

# ── TLV framing ───────────────────────────────────────────────────────────────
# SYNC(1) | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC8(1) | END(1)
# PACKET_SIZE is gone — it was a leftover from the fixed-6-byte line
# protocol that predates stream_parser.py. Frame length is now carried
# in the LEN byte, so nothing needs a compile-time size.

START_BYTE = 0xAA
END_BYTE = 0x55

# ── Panel hardware layout ─────────────────────────────────────────────────────
# These are the ONLY numbers the STM32 and the PC must agree on. They are
# facts about the board, not about what any button means — meaning lives
# entirely on the PC side. Rebinding a button or adding a menu changes
# nothing here and needs no firmware rebuild.

SLOT_COUNT   = 10   # buttons[] / btn_db[] in main.c
POT_COUNT    = 4    # ADC channels 9, 15, 6, 7 — in that order on the wire

# Pixel count and colours are NOT here — the strip is the firmware's
# business. Python sends the menu index and main.c decides what that
# looks like, so restyling the panel is a firmware concern and
# rebinding a button is a Python one.

# Payload sizes, derived. Handy for asserting against LEN on decode.
PANEL_STATE_PAYLOAD_LEN = 2 + (POT_COUNT * 2)    # 10
LED_STATE_PAYLOAD_LEN   = 1                      # menu index

# ── Button slots ──────────────────────────────────────────────────────────────
# Index into main.c's buttons[] table. All ten are active-low and all ten
# are debounced every loop iteration, including the two spares that the
# old firmware sampled and then ignored because no handler was wired to
# them.
#
# NOTE — main.c contradicts itself on the RS labels. The buttons[] table
# says slot 4 is "RS3 = UP" and slot 9 is "RS5 = UP"; the per-menu code
# comments further down say the opposite. The indices below come from the
# buttons[] table, which sits next to the actual pin assignments. If the
# top rocker turns out to do the bottom rocker's job, swap the RS3/RS5
# names here and nothing else changes.
#
# Same for the menu buttons: buttons[] comments say slot 0 is DOWN and
# slot 2 is UP, but the shipped code wires slot 0 to onUpMenuSelect and
# slot 2 to onDownMenuSelect. The code wins below.

SLOT_MENU_UP   = 0   # PA6   (the table comment saying PF12 is stale)
SLOT_SPARE_1   = 1   # PD14  — wired, debounced, never bound
SLOT_MENU_DOWN = 2   # PD15
SLOT_SPARE_3   = 3   # PC7   — wired, debounced, never bound

SLOT_RS3_UP    = 4   # PE10
SLOT_RS4_DOWN  = 5   # PE11
SLOT_RS3_DOWN  = 6   # PE14
SLOT_RS4_UP    = 7   # PE12
SLOT_RS5_DOWN  = 8   # PD11
SLOT_RS5_UP    = 9   # PD13

# The discrete GPIO LEDs are gone — the ws2812 strip is the only
# indicator. That also retired two hardware problems in main.c: leds[]
# was declared [10] with eleven initializers, and leds[2] was PE11, the
# same pin buttons[5] reads as an input.

# ── GCS status indicator ──────────────────────────────────────────────────────
# Pixel 0 of the strip is a link-status light. Nothing about it is
# configured or driven from here — colours, blink rates, breathing, and
# the state machine that picks between them are all in main.c's
# Gcs_Render() and Gcs_Current_State().
#
# The PC sends no status at all. Link state is derived in firmware from
# LED_STATE arrival times: never received = amber breathe, received then
# stopped = red blink, currently arriving = green flash then solid white.
# That's the only arrangement that can work, since a PC that has stopped
# running cannot send a frame saying so.

# ── Panel timing ──────────────────────────────────────────────────────────────

PANEL_FRAME_PERIOD_S = 0.01   # STM32 sends PANEL_STATE at 100 Hz
# LED_STATE resend floor, on top of on-change. This doubles as the link
# keepalive the firmware times out against — main.c's LINK_TIMEOUT_MS is
# 600 ms, so three frames can go missing before the panel calls it lost.
# Raising this means raising LINK_TIMEOUT_MS too.
LED_HEARTBEAT_S = 0.2

MAIN_LOOP_DELAY = 0.01

DEFAULT_TAKEOFF_ALTITUDE = 50
SAFE_BATTERY_LEVEL = 20

# ── Legacy MAVLink-side bit positions ─────────────────────────────────────────
# These are NOT panel bits and never were — they don't match anything in
# the old bit_definitions.py (which had ARM_BIT = 0, not 4). Nothing in
# the files reviewed so far imports them. Left in place because
# commands/translator.py hasn't been checked yet; delete once confirmed.

ARM_BIT = 4
RTL_BIT = 2
MANUAL_BIT = 11
TAKEOFF_BIT = 6
EMERGENCY_BIT = 0
SYSTEM_CHECK_BIT = 8

ENABLE_PREFLIGHT_CHECKS = True
ENABLE_FAILSAFE = True
ENABLE_ARMING_CHECKS = True

STM32_RECONNECT_DELAY = 2
STM32_TIMEOUT = 3

DEBUG_MODE = True
PRINT_RAW_PACKETS = True
PRINT_SYSTEM_STATE = True
# serial_controller/panel_bindings.py
#
# TEMPORARY ADAPTER — delete this file once API/config.py's bindings
# carry explicit slot numbers.
#
# API/config.py is already the source of truth for menus, labels, and
# behaviours. The one thing it doesn't have yet is which physical slot
# each binding sits on, and its positional order can't be used to infer
# that: menu 1 pads with _free(WHITE) at the END of the tuple rather
# than in the slot position, so index 3 is ir_camera_dzoom_plus there
# and wideout in menu 0. Two different slots, same index.
#
# So the slot numbers live here for now, keyed by the same menu bit and
# the same `field` strings API/config.py already uses. When config.py
# grows `slot=`, this file goes away and input_router.py reads CONFIG
# directly — nothing else changes, because everything downstream already
# works in field names.
#
# Every binding below is taken from the per-menu blocks in the OLD
# main.c, by slot index rather than by the RS labels in its comments
# (which contradict the buttons[] table — see system_config.py).

from dataclasses import dataclass

from state.system_config import (
    SLOT_RS3_UP, SLOT_RS3_DOWN,
    SLOT_RS4_UP, SLOT_RS4_DOWN,
    SLOT_RS5_UP, SLOT_RS5_DOWN,
)


# ── Behaviours ────────────────────────────────────────────────────────────────
# What a press does. The firmware has no equivalent — it only reports the
# debounced level, and all latching happens on this side now.

@dataclass(frozen=True)
class Momentary:
    """Active only while held. Mirrors main.c's Set_Bit_From_Debounced."""
    field: str


@dataclass(frozen=True)
class Toggle:
    """Flips a latched boolean on each rising edge. Mirrors main.c's
    Poll_Debounced + an on*_Button_Press that XORs one bit."""
    field: str


@dataclass(frozen=True)
class PairToggle:
    """One latched boolean, two field names — emits field_on when it turns
    on and field_off when it turns off. This is the Record control:
    API/config.py models it as Toggle(field_on="start_record",
    field_off="stop_record"), and PayloadCommand had no single "is
    recording" boolean, which is why panel_sync.py had to reconstruct one
    from two momentary pulses. The latch lives here now, so that
    reconstruction can eventually go."""
    field_on: str
    field_off: str

    @property
    def field(self) -> str:
        # State key, matching API/state.py's _key() which picks field_on.
        return self.field_on


@dataclass(frozen=True)
class Radio:
    """Rising edge selects this field and clears every other field in the
    same group.

    Replaces main.c's hand-written mutual-exclusion pairs, which had a
    real bug: onAI_TRACKING_Button_Press reads

        if (PAYLOAD & JOYSTICK_TRACK) PAYLOAD &= ~JOYSTICK_TRACK;
        if (PAYLOAD & AI_TRACKING)    PAYLOAD &= ~AI_TRACKING;
        else                          PAYLOAD |= AI_TRACKING;

    The `else` binds to the SECOND `if`, so when both bits happen to be
    set the handler clears both and sets neither. The laser cont/single
    pair has the identical shape. Expressing it as a group removes the
    possibility."""
    field: str
    group: str


# ── Bindings ──────────────────────────────────────────────────────────────────
# {menu bit: {slot: behaviour}}. Menu bits match API/config.py's Menu.bit.

BINDINGS = {
    # bit 0 — zoom_fov_focus. All six slots, all momentary.
    0: {
        SLOT_RS3_UP:   Momentary("zoomin"),
        SLOT_RS3_DOWN: Momentary("zoomout"),
        SLOT_RS4_UP:   Momentary("widein"),
        SLOT_RS4_DOWN: Momentary("wideout"),
        SLOT_RS5_UP:   Momentary("focus_in"),
        SLOT_RS5_DOWN: Momentary("focus_out"),
    },

    # bit 1 — picture_select. RS4-DOWN unused, matching _free(WHITE).
    #
    # NOTE: the old main.c bound ir_dzoom_plus to slot 8 and ir_dzoom_minus
    # to slot 9, the opposite polarity to every other menu (where slot 9 is
    # UP). Preserved exactly as the firmware behaved rather than silently
    # "fixing" it — but if IR digital zoom feels inverted on the panel,
    # this is the line to swap.
    1: {
        SLOT_RS3_UP:   Momentary("image_sensor_change"),
        SLOT_RS3_DOWN: Momentary("ir_polarity"),
        SLOT_RS4_UP:   Toggle("near_infrared_toggle"),
        SLOT_RS5_DOWN: Momentary("ir_camera_dzoom_plus"),
        SLOT_RS5_UP:   Momentary("ir_camera_dzoom_minus"),
    },

    # bit 2 — tracking.
    #
    # joystick_track has a firmware handler in the old main.c but was
    # never called from any menu block, so it was unreachable. It's the
    # other half of the track_mode group and wants a slot; RS4-DOWN is
    # free here. Left unbound to match shipped behaviour — uncomment to
    # make it reachable.
    2: {
        SLOT_RS3_UP:   Toggle("tracking_source_toggle"),
        SLOT_RS3_DOWN: Toggle("tracking_search_on_off"),
        SLOT_RS4_UP:   Radio("ai_tracking_on_off", group="track_mode"),
        # SLOT_RS4_DOWN: Radio("joystick_track", group="track_mode"),
        SLOT_RS5_UP:   Toggle("tracking_template_toggle"),
    },

    # bit 3 — laser.
    3: {
        SLOT_RS3_UP:   Toggle("laser_on_off"),
        SLOT_RS4_UP:   Radio("laser_single_mode", group="laser_mode"),
        SLOT_RS4_DOWN: Radio("laser_cont_mode",   group="laser_mode"),
        SLOT_RS5_UP:   Momentary("laser_zoom_in"),
        SLOT_RS5_DOWN: Momentary("laser_zoom_out"),
    },

    # bit 4 — capture. "strobe" was a USB_MESSAGE bit rather than a
    # PAYLOAD_COMMAND one in the old firmware; that distinction no longer
    # exists, since there's one slot bitmap and no registers at all.
    4: {
        SLOT_RS3_UP:   Momentary("take_picture"),
        SLOT_RS3_DOWN: PairToggle(field_on="start_record", field_off="stop_record"),
        SLOT_RS4_UP:   Momentary("motor_on_off"),
        SLOT_RS4_DOWN: Momentary("picture_record_mode_toggle"),
        SLOT_RS5_UP:   Toggle("strobe"),
    },

    # bit 5 — display.
    #
    # RS5-UP calls the same tracking_template_toggle as menu 2 did. That
    # looks like a copy-paste leftover (API/config.py flags it too), and
    # ir_rainbow has a handler that nothing reaches. Preserved as-is;
    # change the field name here to make ir_rainbow reachable.
    5: {
        SLOT_RS3_UP:   Toggle("video_ip"),
        SLOT_RS3_DOWN: Toggle("eo_image_on_off"),
        SLOT_RS4_DOWN: Toggle("eo_dzoom_toggle"),
        SLOT_RS5_UP:   Toggle("tracking_template_toggle"),
    },
}


def bindings_for(menu_bit: int) -> dict:
    """Slot -> behaviour for one menu. Empty dict for an unconfigured
    menu, so an out-of-range index degrades to 'no buttons do anything'
    rather than raising in the serial processor thread."""
    return BINDINGS.get(menu_bit, {})


def all_fields() -> set:
    """Every field name any menu can produce. main.py can assert this
    against the translator's dispatch table at startup, so a typo stops
    the app instead of waiting for someone to press that button."""
    out = set()
    for menu in BINDINGS.values():
        for behaviour in menu.values():
            if isinstance(behaviour, PairToggle):
                out.add(behaviour.field_on)
                out.add(behaviour.field_off)
            else:
                out.add(behaviour.field)
    return out

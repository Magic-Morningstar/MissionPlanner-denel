"""The control panel layout.

This is the data. Edit here to add or rebind a control; model.py only
changes if the *shape* of a binding changes.
"""

from .model import (
    Button, Hold, Status, Toggle, Unassigned,
    FieldBinding, Menu, MenuConfig,
)

RED, GREEN, BLUE = Button.RED, Button.GREEN, Button.BLUE
YELLOW, WHITE, BLACK = Button.YELLOW, Button.WHITE, Button.BLACK

PROPOSED = Status.PROPOSED


def _f(field: str, label: str) -> FieldBinding:
    """Shorthand for a plain protocol field."""
    return FieldBinding(field=field, label=label)


def _free(*buttons: Button) -> tuple[Unassigned, ...]:
    """Shorthand for button slots that exist but do nothing yet."""
    return tuple(Unassigned(button=b) for b in buttons)


CONFIG = MenuConfig(
    default_menu_id="zoom_fov_focus",
    menus=(
        # bit 0 — unchanged. All 6 buttons used.
        Menu(
            id="zoom_fov_focus",
            bit=0,
            title="Zoom / FOV / Focus",
            short_title="ZOOM",
            bindings=(
                _f("zoomin", "Zoom In"),
                _f("zoomout", "Zoom Out"),
                _f("widein", "FOV Narrow"),
                _f("wideout", "FOV Wide"),
                _f("focus_in", "Focus Near"),
                _f("focus_out", "Focus Far"),
            ),
        ),

        # bit 1 — RS4-DOWN (WHITE-equivalent slot) is unused in this
        # block; unchanged otherwise.
        Menu(
            id="picture_select",
            bit=1,
            title="Picture Select / IR Zoom",
            short_title="IMAGE",
            bindings=(
                _f("image_sensor_change", "Image Sensor"),
                _f("ir_polarity", "IR Polarity"),
                _f("near_infrared_toggle", "Near IR"),
                _f("ir_camera_dzoom_plus", "IR DZoom +"),
                _f("ir_camera_dzoom_minus", "IR DZoom \u2212"),
                *_free(WHITE),
            ),
        ),

        # bit 2 — CHANGED. Only 4 real bindings now; two button slots
        # are unused in this block (RS4-DOWN and RS5-UP are not
        # referenced anywhere in the tracking menu's code this time).
        Menu(
            id="tracking",
            bit=2,
            title="Tracking",
            short_title="TRACK",
            bindings=(
                _f("tracking_source_toggle", "Track Source"),
                _f("tracking_search_on_off", "Tracking Search"),
                _f("tracking_template_toggle", "Template"),
                _f("ai_tracking_on_off", "AI Tracking"),
                *_free(WHITE, RED),
            ),
        ),

        # bit 3 — laser_on_off is back inside this menu (not a separate
        # "system" menu — there isn't one in this firmware). One button
        # slot (RS5-UP) unused.
        Menu(
            id="laser",
            bit=3,
            title="Laser",
            short_title="LASER",
            bindings=(
                _f("laser_cont_mode", "Laser Continuous"),
                _f("laser_single_mode", "Laser Single"),
                _f("laser_zoom_in", "Laser Zoom In"),
                _f("laser_zoom_out", "Laser Zoom Out"),
                _f("laser_on_off", "Laser On/Off"),
                *_free(YELLOW),
            ),
        ),

        # bit 4 — CHANGED. "strobe" added (RS3-UP) — this is a
        # ButtonState/USB_MESSAGE field, not a PayloadCommand field
        # like everything else in this menu, even though it's gated by
        # menu selection the same way. See the note below the CONFIG
        # block about what else this needs. One button slot (RS3-DOWN)
        # unused.
        Menu(
            id="capture",
            bit=4,
            title="Capture",
            short_title="CAPTURE",
            bindings=(
                _f("take_picture", "Take Picture"),
                Toggle(
                    button=YELLOW,
                    label="Record",
                    firmware_handler="onRECORD_Button_Press",
                    field_on="start_record",
                    field_off="stop_record",
                    command_on="StartRecordCommand",
                    command_off="StopRecordCommand",
                ),
                Hold(
                    button=WHITE,
                    label="Photo/Video Mode",
                    firmware_bit="PAYLOAD_BIT_PIC_RECORD_MODE_TOGGLE",
                    field="picture_record_mode_toggle",
                    command_press="PictureRecordModeToggleCommand",
                ),
                Hold(
                    button=BLACK,
                    label="Stropes",
                    firmware_bit="PAYLOAD_BIT_MOTOR_ON_OFF",
                    field="motor_on_off",
                    command_press="MotorToggleCommand",
                ),
                _f("strobe", "Strobes"),
                *_free(RED),
            ),
        ),

        # bit 5 — CHANGED, and this one needs your attention: the 4th
        # binding (RS3-UP) calls onTrackingTemplateToggle_Button_Press
        # in the firmware — the exact same handler the tracking menu
        # (bit 2) already uses on its own RS3-UP. onIRRainbow_Button_Press
        # still exists in the firmware but isn't called from anywhere
        # anymore, so ir_rainbow is currently unreachable from any
        # button. Reflecting the code exactly as written below (with
        # tracking_template_toggle listed here too) rather than
        # guessing this was supposed to say ir_rainbow — but this reads
        # like a copy-paste leftover, not an intentional duplicate.
        Menu(
            id="display",
            bit=5,
            title="Display / Video Source",
            short_title="DISPLAY",
            bindings=(
                _f("video_ip", "Video Source"),
                _f("eo_image_on_off", "EO Image"),
                _f("eo_dzoom_toggle", "EO DZoom"),
                _f("tracking_template_toggle", "Template"),  # see note above
                *_free(BLACK, RED),
            ),
        ),
    ),
)
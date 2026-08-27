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
    default_menu_id="display",
    menus=(
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
                *_free(YELLOW),
            ),
        ),
        Menu(
            id="tracking",
            bit=2,
            title="Tracking",
            short_title="TRACK",
            bindings=(
                _f("tracking_search_on_off", "Tracking Search"),
                _f("joystick_track", "Joystick Track"),
                _f("ai_tracking_on_off", "AI Tracking"),
                _f("tracking_template_toggle", "Template"),
                _f("tracking_source_toggle", "Track Source"),
                *_free(RED),
            ),
        ),
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
                *_free(GREEN, YELLOW),
            ),
        ),
        Menu(
            id="capture",
            bit=4,
            title="Capture",
            short_title="CAPTURE",
            status=PROPOSED,
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
                    firmware_bit="BIT_PICTURE_RECORD_MODE_TOGGLE",
                    field="picture_record_mode_toggle",
                    command_press="PictureRecordModeToggleCommand",
                ),
                Hold(
                    button=BLACK,
                    label="Gimbal Motors",
                    firmware_bit="BIT_MOTOR_TOGGLE",
                    field="motor_on_off",
                    command_press="MotorToggleCommand",
                ),
                *_free(BLUE, RED),
            ),
        ),
        Menu(
            id="display",
            bit=5,
            title="Display / Video Source",
            short_title="DISPLAY",
            status=PROPOSED,
            bindings=(
                _f("video_ip", "Video Source"),
                _f("eo_image_on_off", "EO Image"),
                _f("eo_dzoom_toggle", "EO DZoom"),
                _f("ir_rainbow", "IR Rainbow"),
                *_free(BLUE, RED),
            ),
        ),
        Menu(
            id="system",
            bit=6,
            title="System / Laser Power",
            short_title="SYSTEM",
            status=PROPOSED,
            bindings=(
                _f("laser_on_off", "Laser Power"),
                *_free(GREEN, WHITE, YELLOW, BLACK, BLUE),
            ),
        ),
    ),
)
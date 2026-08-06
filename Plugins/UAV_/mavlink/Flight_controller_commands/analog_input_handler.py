# mavlink/Flight_controller_commands/analog_input_handler.py
#
# Flight-only now. The payload half (gimbal rate, tracking search nudge,
# zoom/focus edge triggers) moved to payload_analog_input_handler.py's
# PayloadAnalogInputHandler, wired to PayloadCommandSender instead of a
# single combined sender. All that's left here is joystick -> FBWA/FBWB
# manual-control streaming, which only ever needed manual_ctrl
# (ManualController, flight-side) — it never actually used `sender`.

import logging

logger = logging.getLogger(__name__)


class AnalogInputHandler:
    """
    Starts ManualController streaming when the vehicle enters FBWA/FBWB —
    ManualController itself reads joystick state directly at 10Hz once
    streaming, so nothing needs to be passed in per-call here.
    """

    def __init__(self, state, manual_ctrl):
        self.state       = state
        self.manual_ctrl = manual_ctrl   # ManualController instance

    def process(self):
        current_mode = self.state.get_UAV_Current_Mode
        self._handle_joystick(current_mode)

    def _handle_joystick(self, mode):
        if mode in ("FBWA", "FBWB"):
            if self.manual_ctrl and not self.manual_ctrl._streaming:
                self.manual_ctrl.start_streaming()
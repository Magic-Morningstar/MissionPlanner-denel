# serial_controller/status_builder.py
#
# Replaces packet_builder.py. Reads UAV FACTS from SystemState (armed,
# mode, flying — these are legitimate shared facts, not serial internals,
# so reading them here is fine) and assembles the outgoing StatusUpdate.
# Does NOT read the raw incoming register — the old bug where the
# outgoing message was built by mutating the incoming one is gone
# because there's no longer a shared int at all.

from serial_controller.protocol.messages import StatusUpdate
import logging
logger = logging.getLogger(__name__)


class StatusBuilder:

    def __init__(self, state):
        self.state = state

    def build(self) -> StatusUpdate:
        status = StatusUpdate(
            armed       = self.state.is_UAV_Armed,
            manual_mode = False,
            auto_mode   = False,
            autoland_mode = False,
            is_flying   = self.state.is_flying,
        )
        logger.info(f"StatusBuilder: {status}")
        return status

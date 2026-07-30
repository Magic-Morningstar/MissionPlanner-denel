from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
print("Listening for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (master.target_system, master.target_component))

def _send_raw_gimbal_frame(drone,self, frame: bytes, expect_response=False):
    """
    Tunnel a raw Viewpro protocol frame to the gimbal through the FC's
    dedicated serial port, via MAVLink SERIAL_CONTROL. The FC does not
    parse or validate these bytes — it just writes them out the target
    UART, so this bypasses DO_MOUNT_*/camera-command semantics entirely.
    """
    if not self._check(self._requires_payload_connection):
        return False

  
    flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
    if expect_response:
        flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND

    self.state.mav_connection.mav.serial_control_send(
        device=mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM2,
        flags=flags,
        timeout=0,
        baudrate=0,  
        count=len(frame),
        data=bytes(frame).ljust(70, b'\x00')
    )

    return True
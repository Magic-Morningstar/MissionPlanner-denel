import serial
import serial.tools.list_ports
from utils.helper import *
import config


class SerialHandler:
    def __init__(self, STATE):
        self.ser = None
        self.state = STATE

    def find_stm_port(self):
        """
        Scans connected devices and returns the COM port matching
        the STM32's USB Vendor ID and Product ID — reliable across
        any PC regardless of driver naming differences.
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == config.CONTROLLER_USB_VID and port.pid == config.CONTROLLER_USB_PID:
                system_Print(f"Found STM32 on {port.device} (VID:PID {port.vid:04X}:{port.pid:04X})")
                return port.device

        system_Print("No STM32 device found on any COM port.")
        return None

    def connect(self):
        try:
            detected_port = self.find_stm_port()

            if detected_port is None:
                self.state.update_serialConnection(False, self.ser)
                return False

            self.ser = serial.Serial(
                port=detected_port,
                baudrate=config.BAUDRATE,
                timeout=config.SERIAL_TIMEOUT
            )

            self.state.update_serialConnection(True, self.ser)
            system_Print(f"Controller connected on {detected_port}")
            system_Print("-" * 40)
            return True
        except serial.SerialException:
            self.state.update_serialConnection(False, self.ser)
            return False

    def disconnect_serial(self):
        """Close serial port cleanly"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.state.update_serialConnection(False, self.ser)
        system_Print("Controller disconnected")
        system_Print("-" * 40)

    def read_line(self):
        try:
            if self.ser is None:
                system_Print("connection is lost")
                return None

            if self.ser.in_waiting > 0:
                return self.ser.readline().decode('utf-8').strip()

            # Nothing new in the buffer — not an error, just no data yet
            return None

        except serial.SerialException:
            system_Print("Connection lost!")
            self.disconnect_serial()

            return None

    def send(self, data: bytes):
        try:
            if self.ser:
                self.ser.write(data)
        except serial.SerialException:
            system_Print("Connection lost!")
            self.disconnect_serial()
import serial
from utils.helper import *
import config
class SerialHandler:
    def __init__(self,STATE):
        self.ser = None
        self.state = STATE

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=config.SERIAL_PORT,
                baudrate=config.BAUDRATE,
                timeout=config.SERIAL_TIMEOUT
            )
            self.state.SERIAL_CONNECTION = self.ser
            self.state.update_serialConnection(True)
            system_Print(f"Controller connected on {config.SERIAL_PORT}")
            system_Print("-" * 40)
            return True
        except serial.SerialException:
            self.state.update_serialConnection(False)
            return False

    def disconnect_serial(self):
        """Close serial port cleanly"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.state.update_serialConnection(False)
        system_Print("Controller disconnected")
        system_Print("-" * 40) 

    def read_line(self):
        try:
            if self.ser and self.ser.in_waiting:
                return self.ser.readline().decode('utf-8').strip()
            return None
        except serial.SerialException:
                system_Print("Connection lost!")
                self.disconnect_serial()

    def send(self, data: bytes):
        try:
            if self.ser:
                self.ser.write(data)
        except serial.SerialException:
                system_Print("Connection lost!")
                self.disconnect_serial()
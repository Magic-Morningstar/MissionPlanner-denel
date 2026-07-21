

class GimbalMessageBuilder:

    def __init__(self):
        self.byte0      = 0x55  # Header value 1
        self.byte1      = 0xAA  # Header value 2
        self.byte2      = 0xDC  # Header value 3
        self.byte3      = 0x11  # Number of bytes in the payload including the checksum and byte 3
        self.SERVO_CONTROL = {
            0x00: {
                "label": "Motor ON/OFF",
                "description": "Motor ON/OFF"
            },
            0x01: {
                "label": "Manual speed mode",
                "description": "Manual speed mode"
            },
            0x09: {
                "label": "Manual relative angle mode",
                "description": "Current position is treated as 0°"
            },
            0x0B: {
                "label": "Manual absolute angle mode",
                "description": "Home position is treated as 0°"
            },
        }
        self.message    = bytearray(20)


    def _calculateCheckSum(self):
        pass

    def _calculateByte(self, angle):
        return int((angle / 360.0) * 65536) & 0xFFFF
    
    def calculateAzimuth(self, angle):
        return self._calculateByte(angle = angle)
    
    def calculateTilt(self, angle):
        return self._calculateByte(angle = angle)
    
    def insertWordAt(self, value, position):
        self.message[position] = (value >> 8) & 0xFF      # MSB

    def _calculateSpeed(self,value):
        pass

    


    


     

    

    


        
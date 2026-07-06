import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import threading
import time
from state.system_state import SystemState
from serial_controller.serial_handler import SerialHandler
from serial_controller.packet_Parser import PacketParser
from serial_controller.packet_builder import PacketBuilder
from mavlink.mavlink_contoller import Mavlink_controller
from utils.helper import *
class Controller:

    def __init__(self):
        self.DRONE = None
        self.state = SystemState()
        self.connected = None    
        self.Mavlink_controller = Mavlink_controller(self.state)
        self.SerialHandler = SerialHandler(self.state)


    def start(self):
        self.Mavlink_controller.initiate_Connection()
        self.Mavlink_controller.heartbeat()
        self.Mavlink_controller.update_State()
        self.SerialHandler.connect()
        last_data_time = None
        TIMEOUT = 5
        while True:
            if not self.state.SERIAL_CONNECTION_STATUS:
                if self.SerialHandler.connect():
                    
                    last_data_time = time.time()
                else:
                    system_Print(f"Controller not found — retrying in 1 seconds...")
                    time.sleep(1)
                    continue


            line = self.SerialHandler.read_line()
            data = None
            last_data_time = time.time() 
            if line:
                data = PacketParser.parse(line)
            USB_message = 0

            if not data:
                time.sleep(0.01)
                continue
            if not self.state.CONVERSATION_STARTED:
                self.state.CONVERSATION_STARTED = True
                self.state.PREVIOUS_MESSAGE = data.get("raw")
                system_Print(f"MESSAGE")
                system_Print(f"  Decimal:     {data.get("raw")}")
                system_Print(f"  Hex:         0x{data.get("raw"):08X}")
                system_Print(f"  Binary:      {data.get("raw"):032b}")
                system_Print(f"  Max 32-bit:  {data.get("raw")} / 4294967295")
                system_Print("-" * 40)
                
                
                

                self.Mavlink_controller.execute_instructions(data)
                
                USB_message = PacketBuilder(self.state).serialize()
                USB_message |= data.get("raw")
                PacketParser.print_USBMessage(USB_message)
                self.SerialHandler.send(PacketBuilder(self.state).build(USB_message))
            
            if self.state.PREVIOUS_MESSAGE != data.get("raw"):
                self.state.PREVIOUS_MESSAGE = data.get("raw")
                system_Print(f"-"*40)
                system_Print(f"  Decimal:     {data.get("raw")}")
                system_Print(f"  Hex:         0x{data.get("raw"):08X}")
                system_Print(f"  Binary:      {data.get("raw"):032b}")
                system_Print(f"  Max 32-bit:  {data.get("raw")} / 4294967295")
                system_Print("-" * 40)

                self.Mavlink_controller.execute_instructions(data)
                
                USB_message = PacketBuilder(self.state).serialize()
                USB_message |= data.get("raw")
                
                PacketParser.print_USBMessage(USB_message)
                self.SerialHandler.send(PacketBuilder(self.state).build(USB_message))

if __name__ == "__main__":
    app  = Controller()
    app.start()

                






        


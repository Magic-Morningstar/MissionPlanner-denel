from pymavlink import mavutil
import time

class Battery:

    def __init__(self, drone):
        self.drone = drone

    def get_message(self):
        msg = self.drone.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        return msg

    def battery_voltage(self):
        msg = self.get_message()
        if msg:
            return msg.voltage_battery / 1000.0
        return 0

    def current_battery(self):
        msg = self.get_message()
        if msg:
            return msg.current_battery / 100.0  # convert to amps
        return 0

    def battery_remaining(self):
        msg = self.get_message()
        if msg:
            return msg.battery_remaining
        return 0

    def battery_status(self):
        remaining = self.battery_remaining()
        if remaining < 20:
            print('  WARNING: Battery too low! Aborting.')
            raise SystemExit
        else:
            print('  Battery OK')

    def battery_summary(self):
        msg = self.get_message()
        if not msg:
            print('  No battery data received')
            return

        voltage   = msg.voltage_battery / 1000.0
        current   = msg.current_battery / 100.0
        remaining = msg.battery_remaining

        # determine battery health label
        if remaining > 50:
            health = 'GOOD'
        elif remaining > 20:
            health = 'LOW'
        else:
            health = 'CRITICAL'

        print('╔══════════════════════════════╗')
        print('║       BATTERY SUMMARY        ║')
        print('╠══════════════════════════════╣')
        print('║ Voltage   : ' + str(round(voltage, 2)) + 'V' + ' ' * (16 - len(str(round(voltage, 2)))) + '║')
        print('║ Current   : ' + str(round(current, 2)) + 'A' + ' ' * (16 - len(str(round(current, 2)))) + '║')
        print('║ Remaining : ' + str(remaining) + '%' + ' ' * (16 - len(str(remaining))) + '║')
        print('║ Health    : ' + health + ' ' * (17 - len(health)) + '║')
        print('╚══════════════════════════════╝')


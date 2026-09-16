from pymavlink import mavutil
import time

class GPSChecks:

    def __init__(self, drone):
        self.drone = drone
        self.message = None

    def fetch(self):
        self.message = self.drone.recv_match(
            type='GPS_RAW_INT', blocking=True, timeout=5
        )
        return self.message is not None

    def getFix_type(self, fix_type):
        fix_types = {
            0: 'No GPS',
            1: 'No fix',
            2: '2D fix',
            3: '3D fix',
            4: 'DGPS',
            5: 'RTK float',
            6: 'RTK fixed'
        }
        return fix_types.get(fix_type, 'Unknown')

    def getGPS_Status(self):
        if not self.message:
            print('No GPS message — call fetch() first')
            return
        print('Fix type   : ' + self.getFix_type(self.message.fix_type))
        print('Satellites : ' + str(self.message.satellites_visible))
        print('Latitude   : ' + str(self.message.lat / 1e7) + ' deg')
        print('Longitude  : ' + str(self.message.lon / 1e7) + ' deg')
        print('Altitude   : ' + str(self.message.alt / 1000.0) + ' m')
        print('HDOP       : ' + str(self.message.eph / 100.0))
        print('VDOP       : ' + str(self.message.epv / 100.0))

    def getMessage(self):
        return self.message

    def isReady(self):
        if not self.message:
            return False
        return self.message.fix_type >= 3

    def waitForLock(self):
        print('Waiting for GPS lock...')
        while True:
            self.fetch()
            if self.message:
                print('  Fix: ' + self.getFix_type(self.message.fix_type) +
                      '  Sats: ' + str(self.message.satellites_visible))
                if self.isReady():
                    print('  GPS ready!')
                    return True
            else:
                print('  No GPS message...')
            time.sleep(1)

    def gps_summary(self):
        if not self.message:
            print('No GPS data — call fetch() first')
            return
        fix_str = self.getFix_type(self.message.fix_type)
        sats    = str(self.message.satellites_visible)
        lat     = str(round(self.message.lat / 1e7, 6))
        lon     = str(round(self.message.lon / 1e7, 6))
        alt     = str(round(self.message.alt / 1000.0, 1))
        print('╔══════════════════════════════╗')
        print('║         GPS SUMMARY          ║')
        print('╠══════════════════════════════╣')
        print('║ Fix       : ' + fix_str  + ' ' * (17 - len(fix_str))  + '║')
        print('║ Satellites: ' + sats     + ' ' * (17 - len(sats))     + '║')
        print('║ Latitude  : ' + lat      + ' ' * (17 - len(lat))      + '║')
        print('║ Longitude : ' + lon      + ' ' * (17 - len(lon))      + '║')
        print('║ Altitude  : ' + alt + 'm' + ' ' * (16 - len(alt))     + '║')
        print('╚══════════════════════════════╝')
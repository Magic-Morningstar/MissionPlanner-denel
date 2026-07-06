from pymavlink import mavutil
import time
import sys
sys.path.append('Pre_Flight_Checks')

from .GPS_Checks import GPSChecks

from .bettery_Checks import Battery
class FlightChecks:

    def __init__(self, drone):
        self.drone   = drone
        self.battery = Battery(drone)
        self.gps     = GPSChecks(drone)
        self.all_Pass = False


    def check_connection(self):
        print('[ 1/5 ] Checking connection...')
        msg = self.drone.recv_match(
            type='HEARTBEAT', blocking=True, timeout=5
        )
        if msg:
            print('        Connected! System: ' + str(self.drone.target_system))
            return True
        else:
            print('        FAILED: No heartbeat received')
            return False

    def check_data_streams(self):
        print('[ 2/5 ] Requesting data streams...')
        self.drone.mav.request_data_stream_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1
        )
        time.sleep(2)
        print('        Data streams active')
        return True

    def check_gps(self):
        print('[ 3/5 ] Checking GPS...')
        self.gps.waitForLock()
        self.gps.gps_summary()
        return self.gps.isReady()

    def check_battery(self):
        print('[ 4/5 ] Checking battery...')
        self.battery.battery_summary()
        remaining = self.battery.battery_remaining()
        if remaining < 20:
            print('        FAILED: Battery too low!')
            return False
        print('        Battery OK')
        return True

    def check_ekf(self):
        print('[ 5/5 ] Checking EKF...')
        msg = self.drone.recv_match(
            type='EKF_STATUS_REPORT', blocking=True, timeout=5
        )
        if msg:
            healthy = (msg.flags & 0x1F) == 0x1F
            print('        EKF flags: ' + str(msg.flags))
            if healthy:
                print('        EKF healthy!')
                return True
            else:
                print('        EKF not ready yet')
                return False
        print('        No EKF message received')
        return False

    def run_all_checks(self):
        print('')
        print('╔══════════════════════════════╗')
        print('║     PRE-FLIGHT CHECKS        ║')
        print('╚══════════════════════════════╝')

        results = {
            'Connection'   : self.check_connection(),
            'Data Streams' : self.check_data_streams(),
            'GPS'          : self.check_gps(),
            'Battery'      : self.check_battery(),
            'EKF'          : self.check_ekf(),
        }

        print('')
        print('╔══════════════════════════════╗')
        print('║     CHECK RESULTS            ║')
        print('╠══════════════════════════════╣')
        all_passed = True
        for check, passed in results.items():
            status = 'PASS' if passed else 'FAIL'
            pad = 17 - len(check)
            print('║ ' + check + ' ' * pad + ': ' + status + '         ║')
            if not passed:
                all_passed = False
        print('╠══════════════════════════════╣')
        if all_passed:
            print('║   ALL CHECKS PASSED ✓        ║')
        else:
            print('║   SOME CHECKS FAILED ✗       ║')
        print('╚══════════════════════════════╝')
        print('')

        if not all_passed:
            print('Aborting — fix failed checks before flying')
            return False
            #raise SystemExit
        self.all_Pass = True
        return self.all_Pass
    
    def is_System_Good(self):
        return self.all_Pass
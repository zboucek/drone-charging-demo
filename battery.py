import time
import math
import numpy as np

# Crazylib functions
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

# Parameters
URIS = []
# URIS.append(uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701'))
URIS.append(uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E702'))
URIS.append(uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E703'))
URIS.append(uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E704'))

class Logging:
    """
    Simple logging example class that logs the State Estimate from a supplied
    link uri and disconnects after 5s.
    """
    
    

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        
        self.x = 0
        self.y = 0
        self.z = 0
        self.dt = 0.1
        self.canfly = 0
        self.isflying = 0
        self.crashed = 0
        self.lhstatus = 0
        self.pmstatus = 0

        print('Connecting to %s' % link_uri)
        
        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(
            name='State Estimate', period_in_ms=np.floor(self.dt*1000))

        self._lg_stab.add_variable('lighthouse.status', 'uint8_t')
        self._lg_stab.add_variable('pm.state', 'uint8_t')
        self._lg_stab.add_variable('sys.isFlying', 'uint8_t')
        self._lg_stab.add_variable('sys.canfly', 'uint8_t')
        self._lg_stab.add_variable('sys.isTumbled', 'uint8_t')
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        # # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # t = Timer(10, self._cf.close_link)
        # # Start a timer to disconnect in 10s
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""

        # Get position and velocity
        self.x = data['stateEstimate.x']
        self.y = data['stateEstimate.y']
        self.z = data['stateEstimate.z']
        self.lhstatus = data['lighthouse.status']
        self.pmstatus = data['pm.state']
        self.isflying = data['sys.isFlying']
        self.canfly = data['sys.canfly']
        self.crashed = data['sys.isTumbled']

        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def preflight():
    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0.0
    initial_y = 0.0
    initial_z = 0.0
    initial_yaw = 0  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    time.sleep(1)

    initial_x = np.mean(x_history[:50])
    initial_y = np.mean(y_history[:50])
    initial_z = np.mean(z_history[:50])

    return initial_x, initial_y, initial_z, initial_yaw


def distance(current, goal):
    """Return distance to goal"""
    return np.linalg.norm(current-goal, ord=2)
    #return np.sqrt(np.sum((current-goal)**2))


def on_position(com, goal, yaw):
    d = distance(current_position, goal)
    eps = 0.1
    while d > eps:
        com.send_position_setpoint(goal[0], goal[1], goal[2], yaw)
        time.sleep(0.01)
        d = distance(current_position, goal)

if __name__ == '__main__':
    
    into_air = True    # True if you want to fly the drone

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = Logging(URIS[0])
    
    while True:
        # print('running')
        time.sleep(0.5)

    # set initial points
    initial_x, initial_y, initial_z, initial_yaw = preflight()
    initial = np.array([initial_x, initial_y, initial_z])
    initial_air = np.array([initial_x, initial_y, initial_z+0.5])

    # with SyncCrazyflie(URIS[0], cf=Crazyflie(rw_cache='./cache')) as scf:
    #     # set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
    #     # reset_estimator(scf)
    #     N = 800
    #     dyaw = 0.0

    #     current_x = np.zeros(nx)
    #     current_y = np.zeros(nx)
    #     current_z = np.zeros(nx)

    #     # We take off when the commander is created
    #     com = scf.cf.commander
    #     time.sleep(1)

    #     print("Commander operational, taking off!")

    #     if into_air:
    #         # Take off to inital air position
    #         on_position(com, initial_air, 0)
    #         # To the start of trajectory
    #         print("In the air.")

    #     print("End of mission.")
    #     # End of mission, going for landing!
    #     if into_air:
    #         # RTH
    #         print("RTH.")
    #         on_position(com, initial_air, 0)
    #         # land
    #         print("Landing.")
    #         com.send_velocity_world_setpoint(0, 0, -0.05, dyaw) # landing 0.05 m/s
    #         time.sleep(5)
    #         # Stop motors
    #         print("Stopping motors.")
    #         com.send_stop_setpoint()

    #     print("Closing connection.")
    #     le._cf.close_link()

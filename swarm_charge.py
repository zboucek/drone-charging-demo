# swarm control
from cflib.crazyflie.swarm import *
# for traj
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
# others
import numpy as np
import time
import sys
import datetime
import os
# speech
import playsound
from gtts import gTTS 

SwarmState = namedtuple('SwarmState', 'pmstate lhstatus isflying canfly crashed')

figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()
    
def from_main():
        trajectory_id = 1

        activate_high_level_commander(cf)
        # activate_mellinger_controller(cf)
        duration = upload_trajectory(cf, trajectory_id, figure8)
        print('The sequence is {:.1f} seconds long'.format(duration))
        reset_estimator(cf)
        run_sequence(cf, trajectory_id, duration)

class SwarmCharge(Swarm):
    
    def __init__(self, uris, height = 0.5, factory=None, lang = 'en', sound = True):
        super().__init__(uris, factory)
        self.height = height
        self.cf_in_air = False
        self._states = dict()
        self.t_takeoff = 2.0
        self.t_land = 0.5
        self.t_goto = 5.0
        self.t_wait = 1.0
        self.lang = lang
        self.sound = sound
        
    def __get_charging_status(self, scf):
        log_config = LogConfig(name='state', period_in_ms=10)
        log_config.add_variable('pm.state', 'uint8_t')
        log_config.add_variable('lighthouse.status', 'uint8_t')
        log_config.add_variable('sys.isFlying', 'uint8_t')
        log_config.add_variable('sys.canfly', 'uint8_t')
        log_config.add_variable('sys.isTumbled', 'uint8_t')

        with SyncLogger(scf, log_config) as logger:
            for entry in logger:
                pmstate = entry[1]['pm.state']
                lhstatus = entry[1]['lighthouse.status']
                isflying = entry[1]['sys.isFlying']
                canfly = entry[1]['sys.canfly']
                crashed = entry[1]['sys.isTumbled']
                self._states[scf.cf.link_uri] = SwarmState(pmstate, lhstatus, isflying, canfly, crashed)
                break
    
    
    def get_charging_status(self):
        self.parallel_safe(self.__get_charging_status)
        return self._states
    
    def __get_charged_drone(self):
        """Sequentially search for charged and flyable drone."""
        
        self.get_charging_status()
        for uri, cf in self._cfs.items():
            # return uri, cf
            if (self._states[uri].pmstate == 2 or self._states[uri].pmstate == 2) and self._states[uri].lhstatus == 2:
                return uri, cf
        
        return None, None
    
    def __set_initial_position(self, scf, x=0.0, y=0.0, z=0.0, yaw_radians=0.0):
        scf.cf.param.set_value('kalman.initialX', x)
        scf.cf.param.set_value('kalman.initialY', y)
        scf.cf.param.set_value('kalman.initialZ', z)
        scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

    def __in_air(self):
        if self.cf_in_air:
            return True
        self.get_charging_status()
        for state in self._states:
            if self._states[state].isflying != 0:
                return True
        
        return False
    
    def upload_trajectory(self, cf, trajectory_id, trajectory):
        trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        trajectory_mem.trajectory = []

        total_duration = 0
        for row in trajectory:
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])
            trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
            total_duration += duration

        upload_result = trajectory_mem.write_data_sync()
        if not upload_result:
            print('Upload failed, aborting!')
            sys.exit(1)
        cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
        return total_duration
    
    def __wait_for_position_estimator(self, scf):
        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        # log_config.add_variable('kalman.varPX', 'float')
        # log_config.add_variable('kalman.varPY', 'float')
        # log_config.add_variable('kalman.varPZ', 'float')
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        N = 5
        x_history = [0] * N
        y_history = [0] * N
        # var_y_history = [N] * 10
        # var_x_history = [N] * 10
        # var_z_history = [N] * 10

        threshold = 0.01

        with SyncLogger(scf, log_config) as logger:
            for i, log_entry in enumerate(logger):
                data = log_entry[1]

                x_history.append(data['stateEstimate.x'])
                x_history.pop(0)
                y_history.append(data['stateEstimate.y'])
                y_history.pop(0)
                # var_x_history.append(data['kalman.varPX'])
                # var_x_history.pop(0)
                # var_y_history.append(data['kalman.varPY'])
                # var_y_history.pop(0)
                # var_z_history.append(data['kalman.varPZ'])
                # var_z_history.pop(0)

                # min_x = min(var_x_history)
                # max_x = max(var_x_history)
                # min_y = min(var_y_history)
                # max_y = max(var_y_history)
                # min_z = min(var_z_history)
                # max_z = max(var_z_history)
                # if (max_x - min_x) < threshold and (
                #         max_y - min_y) < threshold and (
                #         max_z - min_z) < threshold and i>N:
                if  i>len(x_history):
                    x = np.mean(x_history)
                    y = np.mean(y_history)
                    return x,y
                    # break

    def __reset_estimator(self, scf):
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        self.__wait_for_position_estimator(scf)
    
    def __tnow(self):
        return datetime.datetime.now().strftime("%H:%M:%S")

    
    def speak(self, text):
        tts = gTTS(text=text, lang=self.lang, slow=False)

        filename = "abc.mp3"
        tts.save(filename)
        playsound.playsound(filename)
        os.remove(filename)

    def msg(self, uri, text):
        if uri is None:
            uri = ""
            drone_number = ""
        else:
            drone_number = "Drone "+uri[-1]
        print(f"[{self.__tnow()}] {uri}: {text}")
        if self.sound:
            self.speak(drone_number+" " +text)
    
    def demo_mission(self):
        if not self.__in_air():
            uri = None
            self.msg(uri, "Waiting for charged drone...")
            while uri is None:
                uri, cf = self.__get_charged_drone()
                time.sleep(1)
            self.msg(uri, "Preflight check")
            x, y = self.__wait_for_position_estimator(cf)
            self.__set_initial_position(cf, x = x, y= y)
            # self.__reset_estimator(cf)
            cf.cf.param.set_value('commander.enHighLevel', '1')
            self.cf_in_air = True
            try:
                commander = cf.cf.high_level_commander
                commander.takeoff(self.height,self.t_takeoff)
                self.msg(uri, "Take off")

                time.sleep(self.t_takeoff+1)
                commander.go_to(x,y,self.height,0.0,self.t_goto)
                self.msg(uri, "Go to setpoint")
                time.sleep(self.t_goto+1)
                self.msg(uri, "Prepare for landing")
                commander.go_to(x,y,0.06,0.0,self.t_goto)
                time.sleep(self.t_goto+2)
                commander.land(0.03,self.t_land)
                self.msg(uri, "Landing...")
                time.sleep(self.t_goto+2)
                time.sleep(self.t_land+1)
                for i in range(5):
                    self.get_charging_status()
                    time.sleep(self.t_wait)
                    if self._states[uri].pmstate == 1:
                        break
                    elif self._states[uri].pmstate == 3:
                        self.msg(uri, "Low battery, abort mission!")
                        break
                while self._states[uri].pmstate != 1:
                    self.msg(uri, "Landing failed, repeat landing")
                    commander.go_to(x,y,0.15,0.0,self.t_goto)
                    time.sleep(self.t_goto+1)
                    commander.go_to(x,y,0.06,0.0,self.t_goto)
                    time.sleep(self.t_goto+2)
                    commander.land(0.03,self.t_land)
                    time.sleep(self.t_goto)
                    time.sleep(self.t_wait)
                    for i in range(5):
                        self.get_charging_status()
                        time.sleep(self.t_wait)
                        if self._states[uri].pmstate == 1:
                            break
                        elif self._states[uri].pmstate == 3:
                            self.msg(uri, "Low battery, abort mission!")
                            break
                
                self.msg(uri, "Landing successful!")
                commander.stop()
                self.cf_in_air = False
            except Exception as e:
                commander.stop()
                self.close_links()
        else:
            # print(f"[{self.__tnow()}] Err: drone in the air!")
            return
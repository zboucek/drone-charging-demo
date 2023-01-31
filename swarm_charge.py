from cflib.crazyflie.swarm import *
from numpy import mean
import time
import datetime
import playsound
from gtts import gTTS 
import os

SwarmState = namedtuple('SwarmState', 'pmstate lhstatus isflying canfly crashed')

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
        log_config.add_variable('pm.batteryLevel', 'uint8_t')
        log_config.add_variable('lighthouse.status', 'uint8_t')
        log_config.add_variable('sys.isFlying', 'uint8_t')
        log_config.add_variable('sys.canfly', 'uint8_t')
        log_config.add_variable('sys.isTumbled', 'uint8_t')

        with SyncLogger(scf, log_config) as logger:
            for entry in logger:
                pmstate = entry[1]['pm.state']
                pmlevel = entry[1]['pm.batteryLevel']
                lhstatus = entry[1]['lighthouse.status']
                isflying = entry[1]['sys.isFlying']
                canfly = entry[1]['sys.canfly']
                crashed = entry[1]['sys.isTumbled']
                self._states[scf.cf.link_uri] = SwarmState(pmstate, pmlevel, lhstatus, isflying, canfly, crashed)
                break
    
    
    def get_charging_status(self):
        self.parallel_safe(self.__get_charging_status)
        return self._states
    
    def __get_charged_drone(self):
        """Sequentially search for charged and flyable drone."""
        
        self.get_charging_status()
        for uri, cf in self._cfs.items():
            # return uri, cf
            if (self._states[uri].pmstate == 2 or self._states[uri].pmstate == 2) and self._states[uri].lhstatus == 2 and not self._states[uri].crashed:
                return uri, cf
        
        return None, None
    
    def get_battery_level(self):
        self.parallel_safe(self.__get_drone_battery_level)
        return self._states
    
    def __get_drone_battery_level(self):
        """Sequentially search for charged and flyable drone according to battery level."""
        
        self.get_charging_status()
        for uri, cf in self._cfs.items():
            # return uri, cf
            if (self._states[uri].pmlevel > 0.8) and self._states[uri].lhstatus == 2 and not self._states[uri].crashed:
                return uri, cf, self._states[uri].pmlevel
        
        return None, None, None
    
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
    
    def __wait_for_position_estimator(self, scf):
        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        N = 5
        x_history = [0] * N
        y_history = [0] * N

        with SyncLogger(scf, log_config) as logger:
            for i, log_entry in enumerate(logger):
                data = log_entry[1]

                x_history.append(data['stateEstimate.x'])
                x_history.pop(0)
                y_history.append(data['stateEstimate.y'])
                y_history.pop(0)
                if  i>len(x_history):
                    x = mean(x_history)
                    y = mean(y_history)
                    return x,y

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
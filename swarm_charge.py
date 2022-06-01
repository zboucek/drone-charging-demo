from cflib.crazyflie.swarm import *

SwarmState = namedtuple('SwarmState', 'pmstate lhstatus isflying canfly crashed')

class SwarmCharge(Swarm):
    
    def __init__(self, uris, factory=None):
        super().__init__(uris, factory)
        self.in_air = False
        self._states = dict()
        
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
    
    def get_charged_drone(self):
        """Sequentially search for charged and flyable drone."""
        
        states = self.get_charging_status()
        for uri, cf in self._cfs.items():
            if self._states[uri].pmstate == 2 and self._states[uri].lhstatus == 2:# and self._states[uri].canfly !=0:
                return uri, cf
        
        return None, None

    def __in_air(self):
        for uri, cf in self._cfs.items():
            if self._states[uri].isflying != 0:
                return True
        
        return False
    
    def get_charging_status(self):
        self.parallel_safe(self.__get_charging_status)
        return self._states
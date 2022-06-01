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
    
    def __in_air(self):
        """
        Execute a function for all Crazyflies in the swarm, in sequence.

        The first argument of the function that is passed in will be a
        SyncCrazyflie instance connected to the Crazyflie to operate on.
        A list of optional parameters (per Crazyflie) may follow defined by
        the args_dict. The dictionary is keyed on URI.

        Example:
        def my_function(scf, optional_param0, optional_param1)
            ...

        args_dict = {
            URI0: [optional_param0_cf0, optional_param1_cf0],
            URI1: [optional_param0_cf1, optional_param1_cf1],
            ...
        }


        self.sequential(my_function, args_dict)

        :param func: the function to execute
        :param args_dict: parameters to pass to the function
        """
        
        for uri, cf in self._cfs.items():
            if self._states[uri]['isflying'] != 0:
                return True
        
        return False
    
    def get_charging_status(self):
        self.parallel_safe(self.__get_charging_status)
        return self._states
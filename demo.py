# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017-2018 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Version of the AutonomousSequence.py example connecting to 10 Crazyflies.
The Crazyflies go straight up, hover a while and land but the code is fairly
generic and each Crazyflie has its own sequence of setpoints that it files
to.

"""
# from calendar import c
# import time
# from collections import namedtuple

import cflib.crtp
# from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from swarm_charge import SwarmCharge
# from cflib.crazyflie.syncLogger import SyncLogger

# Parameters
HEIGHT = 0.5 # z-global [meters]

# Change uris and sequences according to your setup
URI1 = 'radio://0/100/2M/E7E7E7E701'
URI2 = 'radio://0/100/2M/E7E7E7E702'
URI3 = 'radio://0/100/2M/E7E7E7E703'
URI4 = 'radio://0/100/2M/E7E7E7E704'

# List of URIs, comment the one you do not want to fly
uris = {
    # URI1,
    URI2,
    URI3,
    URI4
}

# List of URIs, comment the one you do not want to fly
uris2 = {
    URI1,
    URI2,
    # URI3,
    URI4
}

if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with SwarmCharge(uris2, factory=factory) as swarm:
        while True:
            swarm.demo_mission()
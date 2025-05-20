# -*- coding: utf-8 -*-
#
#     Drone Charging Demo
#
#  Copyright (C) 2025 Zdeněk Bouček
#
#  This program is free software: you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 3
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  Description:
#  This script demonstrates the Drone Charging Demo application, which
#  manages a swarm of Crazyflie drones for charging, takeoff, and flight missions.
#  It includes functionality for trajectory generation and Lighthouse memory management.

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from swarm_charge import SwarmCharge

# Parameters
HEIGHT = 0.5 # z-global [meters]

# Change uris and sequences according to your setup
URI1 = 'radio://0/100/2M/E7E7E7E701'
URI2 = 'radio://0/100/2M/E7E7E7E702'
URI3 = 'radio://0/100/2M/E7E7E7E703'
URI4 = 'radio://0/100/2M/E7E7E7E704'

# List of URIs, comment the one you do not want to fly
uris = {
    URI1,
    # URI2,
    URI3,
    URI4
}

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with SwarmCharge(uris, factory=factory, lang = 'cs', sound = True, center = [0.0, 0.0, 0.0]) as swarm:
        while True:
            swarm.demo_mission()
            # time.sleep(5)
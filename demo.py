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
from calendar import c
import time
from collections import namedtuple

import cflib.crtp
# from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from swarm_charge import SwarmCharge
# from cflib.crazyflie.syncLogger import SyncLogger

CF_IN_AIR = False # safety variable (only one drone at a time)

# Parameters
HEIGHT = 0.5 # z-global [meters]
VEL_Z = -0.5 # landing velocity [m/s]

# Change uris and sequences according to your setup
# URI1 = 'radio://0/100/2M/E7E7E7E701'
URI2 = 'radio://0/100/2M/E7E7E7E702'
URI3 = 'radio://0/100/2M/E7E7E7E703'
# URI4 = 'radio://0/100/2M/E7E7E7E704'

# List of URIs, comment the one you do not want to fly
uris = {
    # URI1,
    URI2,
    URI3,
    # URI4
}

CF_IN_AIR = False # safety variable (only one drone at a time)
    
def set_initial_position(self, scf, x, y, z, yaw_radians = 0.0):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)
    
    cf.commander.send_position_setpoint(0, 0, HEIGHT, 0)


def land(cf, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    print(vz)

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def run_sequence(scf):
    try:
        cf = scf.cf
        # print(cf)
        # print('fly')
        if CF_IN_AIR:
            CF_IN_AIR = False
        else:
            CF_IN_AIR = False

        print(CF_IN_AIR)
        # take_off(cf, sequence[0])
        # for position in sequence:
        #     print('Setting position {}'.format(position))
        #     end_time = time.time() + position[3]
        #     while time.time() < end_time:
        #         cf.commander.send_position_setpoint(position[0],
        #                                             position[1],
        #                                             position[2], 0)
        #         time.sleep(0.1)
        # land(cf, HEIGHT)
    except Exception as e:
        print(e)


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with SwarmCharge(uris, factory=factory) as swarm:
        # swarm.reset_estimators()
        # swarm.parallel(reset_estimator)
        # while True:
            # print("running sequence")
            # swarm.sequential(run_sequence)
            status = swarm.get_charging_status()
            for uri in uris:
                if status[uri].canfly !=0:
                    print(f"{uri} can fly.")
                else:
                    print(f"{uri} cannot fly.")
            swarm.demo_mission()
            # time.sleep(5)
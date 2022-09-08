# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
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
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Example of how to write to the Lighthouse base station geometry
and calibration memory in a Crazyflie
"""
import logging
from threading import Event

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsCalibration
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from read_lighthouse_mem import ReadMem

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class WriteMem:
    def __init__(self, uri, geo_dict, calib_dict):
        self._event = Event()

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            helper = LighthouseMemHelper(scf.cf)

            helper.write_geos(geo_dict, self._data_written)
            self._event.wait()

            self._event.clear()

            helper.write_calibs(calib_dict, self._data_written)
            self._event.wait()

    def _data_written(self, success):
        if success:
            print('Data written')
        else:
            print('Write failed')

        self._event.set()


if __name__ == '__main__':
    uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    new_read = ReadMem(uri)

    # Change uris and sequences according to your setup
    URI1 = 'radio://0/100/2M/E7E7E7E701'
    URI2 = 'radio://0/100/2M/E7E7E7E702'
    URI3 = 'radio://0/100/2M/E7E7E7E703'
    URI4 = 'radio://0/100/2M/E7E7E7E704'

    # List of URIs, comment the one you do not want to fly
    uris = {
        # URI1,
        URI2,
        # URI3,
        URI4
    }

    for uri_string in uris:
        # URI to the Crazyflie to connect to
        uri = uri_helper.uri_from_env(default=uri_string)

        # Initialize the low-level drivers
        cflib.crtp.init_drivers()

        # Note: base station ids (channels) are 0-indexed
        geo_dict = new_read.geo_data
        calib_dict = new_read.calib_data

        WriteMem(uri, geo_dict, calib_dict)

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
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
Simple example that connects to the Crazyflie with a specific address, logs the Stabilizer
and prints it to the console.
"""
import logging
import time
from threading import Timer
import datetime as dt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# change the uri address to your crazyflie address
uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E73')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import numpy as np
import os
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander


class LoggingExample:

    def __init__(self, link_id):
        """ Initialize and run the example with the specified link_id """

        self.count = 0

        # Initialize cf object
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Initialize log variable
        self.logs = np.zeros([100000,12])

        # Fly a square
        self.is_connected = True
        self.fly_square(link_id)

    def _connected(self, link_id):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=40)
        self._lg_stab.add_variable('range.zrange', 'float')
        self._lg_stab.add_variable('range.front', 'uint16_t')
        # self._lg_stab.add_variable('range.back', 'uint16_t')
        # self._lg_stab.add_variable('range.up', 'uint16_t')
        self._lg_stab.add_variable('range.left', 'uint16_t')
        self._lg_stab.add_variable('range.right', 'uint16_t')
        # self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        # self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        # self._lg_stab.add_variable('acc.y', 'float')  # estimated Z coordinate
        # self._lg_stab.add_variable('acc.z', 'float')  # estimated Z coordinate
        


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

    def fly_square(self, id):
        """ Example of simple logico to make the drone fly in a square 
        trajectory at fixed speed"""

        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            # Send position commands
            # with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            with MotionCommander(scf) as mc:
                # pc.forward(1.0)
                data = self.logs[self.count-1][:]
                print(data)
                z = data[0]
                print('z = ', z)
                time.sleep(2)
                # mc.up(0.3, 0.1)
                # mc.up(0.1, 0.1)
                # print('Moving forward')
                mc.start_forward(0.1)
                move_right = 0
                Obstacle_Avoidance = 0
                while(1):
                    time.sleep(0.1)
                    data = self.logs[self.count-1][:]
                    z_new = data[0]
                    front = data[1]
                    left = data[2]
                    right = data[3]
                    # y = data[9]
                    # z = data[10]
                    print('z new = ', z_new,  'front = ', front,  'left = ', left) #, 'y = ', y) #, 'z = ', z)

                    if front < 400 and move_right == 0:
                        print('OBSTACLE AVOIDANCE')
                        mc.start_right(0.1)
                        move_right = 1
                    elif front >= 400 and move_right == 1:
                         time.sleep(1.5)
                         move_right = 0
                    elif front >= 400 and move_right == 0:
                        mc.start_forward(0.1)

                    if left < 300 and Obstacle_Avoidance == 0:
                        print('OBJECT ON LEFT')
                        Obstacle_Avoidance = 1
                    elif left >= 300 and Obstacle_Avoidance == 1:
                        print('GO TO THE INIT POS')
                        time.sleep(1.5)
                        mc.start_left(0.1)
                        time.sleep(4.5)
                        mc.start_forward(0.1)
                        Obstacle_Avoidance = 0


                    if z_new < 280:
                        time.sleep(2)
                        break
                mc.stop()
                # print('Moving left')
                # pc.back(1.0)
                # print('Moving back')
                # pc.right(1.0)
                # print('Moving right')

        self._disconnected

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        
        # Save info into log variable
        for idx, i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]

        self.count += 1

    def _connection_failed(self, link_id, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_id, msg))
        self.is_connected = False

    def _connection_lost(self, link_id, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_id, msg))

    def _disconnected(self, link_id):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_id)
        self.is_connected = False
        
        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(),'logs',filename)
        np.savetxt(filepath, self.logs, delimiter=',')


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers()
    le = LoggingExample(uri)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)

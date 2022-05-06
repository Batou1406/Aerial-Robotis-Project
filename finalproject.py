import logging
import time
import datetime as dt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
import numpy as np
import os
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# change the uri address to your crazyflie address
uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E73')
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


HEIGHT_DRONE = 300
FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3


[start, landingPadDetected, return, startingPadDetected, finish]

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
        #flag
        self.explorationState = start
        self.obstacleDetected = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.front = 0
        self.left = 0
        self.right = 0
        self.back = 0

        self.flySearch(link_id)

    def _connected(self, link_id):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=40)
        self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self._lg_stab.add_variable('range.zrange', 'uint16_t')
        self._lg_stab.add_variable('range.front', 'uint16_t')
        self._lg_stab.add_variable('range.left', 'uint16_t')
        self._lg_stab.add_variable('range.right', 'uint16_t')
        self._lg_stab.add_variable('range.back', 'uint16_t')

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

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        # Save info into log variable
        self.count += 1
        for idx, i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]

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

    def explore(self):
        return None

    def landing(self):      #Fonction called when landing pad is detected below the drone
        

        if self.z > HEIGHT_DRONE*1.1 and self.landing_state == 0:
            MotionCommander.stop()
            self.landing_state = 1

        if self.landing_state == 1:

            if self.direction == FORWARD:                       #Go back to center of landing pad, depending on where the drone came from and update the direction variable
                MotionCommander.back(0.15, velocity = 0.5)      #Check when the drone start going backaward, what's the distance to be in center of the landing pad (I assumed 15cm)
                MotionCommander.start_right(velocity = 0.5)
                self.direction = RIGHT
            elif self.direction == BACKWARD:
                MotionCommander.forward(0.15, velocity = 0.5)
                MotionCommander.start_right(velocity = 0.5)
                self.direction = RIGHT
            elif self.direction == RIGHT:
                MotionCommander.left(0.15, velocity = 0.5)
                MotionCommander.start_forward(velocity = 0.5)
                self.direction = FORWARD
            elif self.direction == LEFT:
                MotionCommander.right(0.15, velocity = 0.5)
                MotionCommander.start_forward(velocity = 0.5)
                self.direction = FORWARD

            self.landing_state = 2


        if self.z > HEIGHT_DRONE*1.1 and self.landing_state == 2:
            MotionCommander.stop()
            self.landing_state = 3
            
        if self.landing_state == 3:

            if self.direction == FORWARD:                       #Go back to center of landing pad, depending on where the drone came from and update the direction variable
                MotionCommander.back(0.15, velocity = 0.5)
            elif self.direction == RIGHT:
                MotionCommander.left(0.15, velocity = 0.5)


            self.direction = 0
            MotionCommander.stop()
            MotionCommander.land()
            self.explorationState = startingPadDetected
            self.landing_state = 0                              #Drone landed on the landing pad, variabe reinint for next call of the function


        return None

    def returnToStart(self):
        return None

    def obstacleAvoidance():
        return None

    def flySearch(self, id):
        """ Example of simple logico to make the drone fly in a square
        trajectory at fixed speed"""
        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            with MotionCommander(scf) as mc:
                while(self.explorationState != finish):
                    data = self.logs[self.count][:]
                    self.x = data[0]
                    self.y = data[1]
                    self.z = data[2]
                    self.front = data[3]
                    self.left = data[4]
                    self.right = data[5]
                    self.back = data[6]
                    self.landing_state = 0
                    self.direction = 0

                    if(self.explorationState == start):
                        self.explore()
                    elif(self.explorationState == landingPadDetected):
                        self.landing()
                    elif(self.explorationState == return):
                        self.returnToStart()
                    elif(self.explorationState == startingPadDetected):
                        self.landing()
                    elif(self.explorationState == finish):
                        self._disconnected
                    else :
                        print("Error : wrong state machine")
        self._disconnected




if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers()
    le = LoggingExample(uri)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)

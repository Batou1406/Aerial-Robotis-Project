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


HEIGHT_DRONE = 600
FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3
TRESHOLD = 300
PAD_TRESHOLD_RISE = 590
PAD_TRESHOLD_FALL = 630
X0 = 0.3
Y0 = 0.3
START_ZONE_X = 1
MAP_SIZE_X = 3
MAP_SIZE_Y = 0.6
EXPLORESPEED = 0.2
LANDINGSPEED = 0.1
TurnRightDict = {
  'FRONT': 'RIGHT',
  'RIGHT': 'BACK',
  'BACK': 'LEFT',
  'LEFT': 'FRONT'
}
TurnBackDict = {
  'FRONT': 'BACK',
  'RIGHT': 'LEFT',
  'BACK': 'FRONT',
  'LEFT': 'RIGHT'
}
dirToOrderDict = {
  'FRONT': [1,0],
  'RIGHT': [0,0],
  'BACK': [1,0],
  'LEFT': [0,1]
}
nextBorderDict = {
  'FIRST_BORDER': 'SECOND_BORDER',
  'SECOND_BORDER': 'THIRD_BORDER',
  'THIRD_BORDER': 'FOURTH_BORDER',
  'FOURTH_BORDER': 'FIFTH_BORDER',
  'FIFT_BORDER': 'SIXTH_BORDER',
  'SIXTH_BORDER': 'LAND',
  'LAND': 'FIRST_BORDER'
}

class LoggingExample:

    def __init__(self, link_id):
        """ Initialize and run the example with the specified link_id """
        self.count = 0
        self.step = 0
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
        self.explorationState = 'START' #[start, landingPadDetected, return, startingPadDetected, finish]
        self.obstacleDetected = None
        self.x = 0
        self.y = 0
        self.z = 0
        self.front = 0
        self.left = 0
        self.right = 0
        self.back = 0
        self.lastPos = [0,0]
        self.goalPos = [0,0]
        self.explorePos = [0,0]
        self.V = [0, None]
        self.V_ref = [0, None]
        self.statusManoeuvre = 1
        self.exploreStatus = 'LEFT'
        self.landingStatus = 'LAND'
        self.padPos = [0,0]
        self.padDetected = False
        self.landingDirection = None
        self.landingState = 'FINDCORNER'

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
        #première zone
        if(self.x < START_ZONE_X):
            self.V_ref = [EXPLORESPEED, 'FRONT']
        else:
            if(self.exploreStatus == 'FRONT'):
                if(self.explorePos[0] > self.x) :
                    self.V_ref = [EXPLORESPEED, 'FRONT']
                else :
                    if(self.y < MAP_SIZE_Y/2.):
                        self.exploreStatus = 'LEFT'
                    else :
                        self.exploreStatus = 'RIGHT'
            elif(self.exploreStatus == 'LEFT'):
                if(self.y < MAP_SIZE_Y) :
                    self.V_ref = [EXPLORESPEED, 'LEFT']
                else :
                    self.exploreStatus = 'FRONT'
                    self.explorePos = [self.x + 0.2, self.y]
            elif(self.exploreStatus == 'RIGHT'):
                if(self.y > 0) :
                    self.V_ref = [EXPLORESPEED, 'RIGHT']
                else :
                    self.exploreStatus = 'FRONT'
                    self.explorePos = [self.x + 0.2]
            else :
                print('wrong state machine : explore')


    def detectPadRise(self) :
        if(self.z < PAD_TRESHOLD_RISE):
            if(self.explorationState == 'START') :
                self.explorationState = 'LANDINGPADDETECTED'
                time.sleep(1)
            elif(self.explorationState == 'RETURN'):
                self.explorationState = 'STARTINGPADDETECTED'
            self.landingDirection = self.V_ref[1]

    def detectPadFall(self) :
        if(self.z > PAD_TRESHOLD_FALL):
            if(self.landingState == 'FINDCORNER') :
                self.landingState = 'GOMIDDLEPAD'

    def detectPadBorder(self):
        if(self.z < PAD_TRESHOLD_RISE or self.z < PAD_TRESHOLD_FALL):
            time.sleep(1)
            if(self.explorationState == 'START') :
                self.explorationState = 'LANDINGPADDETECTED'
            elif(self.explorationState == 'RETURN'):
                self.explorationState = 'STARTINGPADDETECTED'
            elif(self.explorationState == 'LANDINGPADDETECTED' or self.explorationState == 'STARTINGPADDETECTED'):
                self.padDetected = True
                self.landingStatus = nextBorderDict[self.landingStatus]
            else :
                print("wrong state machine : detectPadBoarder")


    def landing10(self,mc):
        mc.land()
        self.explorationState = 'LANDED'

    # def landing(self,mc):
    #     self.detectPadFall()
    #     if(self.landingState ==  'FINDCORNER'):
    #         mc.land()
    #         self.explorationState='LANDED'
    #         #self.V_ref = [LANDINGSPEED, TurnRightDict[self.landingDirection]]
    #     elif(self.landingState ==  'GOMIDDLEPAD'):
    #         self.landingState = 'FINDCORNER'
    #         moveX = 0.15 if((self.landingDirection == 'FRONT') or (self.landingDirection == 'RIGHT')) else -0.15
    #         moveY = 0.15 if((self.landingDirection == 'FRONT') or (self.landingDirection == 'LEFT')) else -0.15
    #         mc.move_distance(moveX, moveY, 0, velocity=LANDINGSPEED)
    #         mc.land()
    #         print('LANDED')
    #         if(self.explorationState == 'LANDINGPADDETECTED'):
    #             self.explorationState = 'LANDED'
    #         else :
    #             self.explorationState = 'FINISH'
    #     else :
    #         print('Wrong state machine : landing')


    def landing(self,mc):
        if(self.padDetected == True):
            self.padDetected == False
            if(self.landingStatus == 'FIRST_BORDER' or self.landingStatus == 'SECOND_BORDER' or self.landingStatus == 'FOURTH_BORDER' or self.landingStatus == 'SIXTH_BORDER'):
                self.padPos += [self.x*dirToOrderDict[self.V_ref[1]][0], self.y*dirToOrderDict[self.V_ref[1]][1]]
                time.sleep(1)
        if(self.landingStatus == 'SECOND_BORDER'):
            mc.move_distance((self.padPos[0]/2.-self.x)*dirToOrderDict[self.V_ref[1]][0], (self.padPos[1]/2.-self.y)*dirToOrderDict[self.V_ref[1]][1], 0,velocity=LANDINGSPEED)
            time.sleep(1)
            self.V_ref = [LANDINGSPEED,  TurnRightDict[self.V_ref[1]]]
            self.landingStatus = 'THIRD_BORDER'
        elif(self.landingStatus == 'FOURTH_BORDER'):
            self.V_ref = [LANDINGSPEED,  TurnBackDict[self.V_ref[1]]]
        elif(self.landingStatus == 'SIXTH_BORDER'):
            self.landingStatus = 'LAND'
            mc.move_distance(self.padPos[0]/2.-self.x, self.padPos[1]/2.-self.y, 0,velocity=LANDINGSPEED)
            mc.land()

    def landing2(self, mc):      #Fonction called when landing pad is detected below the drone
        if self.z > HEIGHT_DRONE*1.1 and self.landing_state == 0:
            mc.stop()
            self.landing_state = 1

        if self.landing_state == 1:

            if self.landingDirection == 'FRONT':                       #Go back to center of landing pad, depending on where the drone came from and update the direction variable
                mc.back(0.15, velocity = 0.5)      #Check when the drone start going backaward, what's the distance to be in center of the landing pad (I assumed 15cm)
                mc.start_right(velocity = 0.5)
                self.landingDirection = 'RIGHT'
            elif self.landingDirection == 'BACK':
                mc.forward(0.15, velocity = 0.5)
                mc.start_right(velocity = 0.5)
                self.landingDirection = 'RIGHT'
            elif self.landingDirection == 'RIGHT':
                mc.left(0.15, velocity = 0.5)
                mc.start_forward(velocity = 0.5)
                self.landingDirection = 'FRONT'
            elif self.landingDirection == 'LEFT':
                mc.right(0.15, velocity = 0.5)
                mc.start_forward(velocity = 0.5)
                self.landingDirection = 'FRONT'

            self.landing_state = 2


        if self.z > HEIGHT_DRONE*1.1 and self.landing_state == 2:
            mc.stop()
            self.landing_state = 3

        if self.landing_state == 3:

            if self.direction == FORWARD:                       #Go back to center of landing pad, depending on where the drone came from and update the direction variable
                mc.back(0.15, velocity = 0.5)
            elif self.direction == RIGHT:
                mc.left(0.15, velocity = 0.5)


            self.direction = 0
            mc.stop()
            mc.land()
            #self.explorationState = startingPadDetected
            self.landing_state = 0                              #Drone landed on the landing pad, variabe reinint for next call of the function


        return None

    # def returnToStart(self, start):
    #     while ( self.logs[self.count][0] > start[0]):
    #         if (self.logs[self.count][3]>400):
    #             LoggingExample.obstacleAvoidance_forward()
    #         else :
    #             MotionCommander.start_forward(velocity=0.2)
    #     if (self.logs[self.count][1] < start[1]):
    #         while (self.logs[self.count][1] < start[1]):
    #             if (self.logs[self.count][4]>400):
    #                 LoggingExample.obstacleAvoidance_left()
    #             else :
    #                 MotionCommander.start_left(velocity=0.2)
    #     else:
    #         while (self.logs[self.count][1] > start[1]):
    #             if (self.logs[self.count][5]>400):
    #                 LoggingExample.obstacleAvoidance_right()
    #             else :
    #                 MotionCommander.start_right(velocity=0.2)
    #     self.explorationState = startingPadDetected
    #     return None
    def obstacleAvoidance_forward(self, mc):
        print('obstacle avoidance')
        print(self.statusManoeuvre)
        if(self.statusManoeuvre == 1):
            if(self.front < TRESHOLD):
                mc.start_left(velocity=0.2)
            else :
                self.goalPos = [self.x, self.y+0.07]
                self.statusManoeuvre = 2
        elif(self.statusManoeuvre == 2):
            if(self.goalPos[1] > self.y):
                mc.start_left(velocity=0.2)
            else:
                self.statusManoeuvre = 3
                self.goalPos = [self.x + 0.3, self.y]
        elif(self.statusManoeuvre == 3):
            if(self.right < TRESHOLD or (self.x < self.goalPos[0] and self.front > TRESHOLD)):
                mc.start_forward(velocity=0.2)
            else:
                self.goalPos = [self.x + 0.1, self.y]
                self.statusManoeuvre = 4
        elif(self.statusManoeuvre == 4):
            if(self.goalPos[0] > self.x):
                mc.start_forward(velocity=0.2)
            else:
                self.statusManoeuvre = 5
                self.goalPos = [self.x, self.lastPos[1]]
        elif(self.statusManoeuvre == 5):
            if(self.right < TRESHOLD):
                self.statusManoeuvre = 3
            elif(self.goalPos[1] < self.y):
                mc.start_right(velocity=0.2)
            else :
                self.obstacleDetected = None
                self.statusManoeuvre = 1


    def obstacleAvoidance_forward2(self, mc):
        y1 = self.logs[self.count][1]
        while (self.logs[self.count][3] < 300):
            mc.start_left(velocity=0.2)
            print(self.logs[self.count][3])
        mc.left(distance_m = 0.07, velocity=0.2)
        mc.forward(distance_m= 0.3,velocity=0.2)
        while (self.logs[self.count][5] < 500):
            print(self.logs[self.count][4])
            mc.start_forward(velocity=0.2)
        mc.forward(distance_m= 0.1,velocity=0.2)
        mc.right(distance_m = -y1+self.logs[self.count][1],velocity=0.2)
        self.obstacleDetected = None

    def obstacleAvoidance_left(self, mc):
        x1 = self.logs[self.count][0]
        while(self.logs[self.count][4] < 300):
            mc.start_forward(velocity=0.2)
        mc.move_distance(distance_x_m= 0.1, distance_y_m = 0, distance_z_m = 0,velocity=0.2)
        mc.move_distance(distance_x_m= 0, distance_y_m = 0.3, distance_z_m = 0,velocity=0.2)
        while(self.logs[self.count][6] < 300):
            mc.start_left(velocity=0.2)
        mc.move_distance(distance_x_m= 0, distance_y_m = 0.1, distance_z_m = 0,velocity=0.2)
        mc.move_distance(distance_x_m= x1-self.logs[self.count][0], distance_y_m = 0, distance_z_m = 0,velocity=0.2)
        self.obstacleDetected = None

    def obstacleAvoidance_right(self,mc):

        x1 = self.logs[self.count][0]
        while (self.logs[self.count][5] < 300):
            print(self.logs[self.count][5])
            mc.start_forward(velocity=0.2)
        mc.forward(distance_m= 0.1, velocity=0.2)
        mc.right(distance_m = 0.3, velocity=0.2)
        while (self.logs[self.count][6] < 400):
            mc.start_right(velocity=0.2)
            print(self.logs[self.count][6])
        print('xm = ', x1-self.logs[self.count][0])
        print('x1 = ', x1)
        print('x_pos', self.logs[self.count][0])
        mc.right(distance_m = 0.1, velocity=0.2)
        mc.back(distance_m= -x1+self.logs[self.count][0], velocity=0.2)
        self.obstacleDetected = None

    def obstacleAvoidance_back(self, mc):
        print('tbd')
        self.obstacleDetected = None

    def obstacleAvoidanceFunc(self,mc):

        if(self.obstacleDetected is None):
            if(self.front < TRESHOLD) :
                self.obstacleDetected = 'FRONT'
            elif(self.left < TRESHOLD) :
                self.obstacleDetected = 'LEFT'
            elif(self.right < TRESHOLD) :
                self.obstacleDetected = 'RIGHT'
            elif(self.back < TRESHOLD) :
                #self.obstacleDetected = 'BACK'
                print('tbd')
            if(self.obstacleDetected is not None):
                self.lastPos = [self.x, self.y]

        if(self.obstacleDetected is not None) :
            if(self.obstacleDetected == 'FRONT'):
                self.obstacleAvoidance_forward(mc)
            if(self.obstacleDetected == 'LEFT'):
                self.obstacleAvoidance_left(mc)
            if(self.obstacleDetected == 'RIGHT'):
                self.obstacleAvoidance_right(mc)
            if(self.obstacleDetected == 'BACK'):
                self.obstacleAvoidance_back(mc)
        elif((self.explorationState != 'LANDED') and (self.explorationState != 'FINISH')) :
            if(self.V_ref[1] == 'FRONT') :
                mc.start_forward(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'LEFT') :
                 mc.start_left(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'RIGHT') :
                 mc.start_right(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'BACK') :
                 mc.start_back(velocity=self.V_ref[0])

    def updateSensorValue(self):
        data = self.logs[self.count][:]
        self.x = data[0] + X0
        self.y = data[1] + Y0
        self.z = data[2]
        self.front = data[3]
        self.left = data[4]
        self.right = data[5]
        self.back = data[6]

    def flySearch(self, id):
        """ Example of simple logico to make the drone fly in a square
        trajectory at fixed speed"""
        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            with MotionCommander(scf) as mc:
                x0 = 0.3
                y0 = 0.45
                mc.up(0.3)
                time.sleep(2)
                # while(self.explorationState != finish):
                while(1):
                    time.sleep(0.1)

                    print(self.explorationState)
                    self.updateSensorValue()
                    self.detectPadBorder()

                    if(self.explorationState == 'START'):
                        #self.explore()
                        self.V_ref = [0.2, 'FRONT']
                    elif(self.explorationState == 'LANDINGPADDETECTED'):
                        self.landing(mc)
                    elif(self.explorationState == 'RETURN'):
                        self.explorationState == 'FINISH'
                        # self.returnToStart()
                    elif(self.explorationState == 'STARTINGPADDETECTED'):
                        self.landing(mc)
                    elif((self.explorationState == 'FINISH') or (self.explorationState == 'LANDED')):
                        self._disconnected
                    else :
                        print("Error : wrong state machine")

                    self.obstacleAvoidanceFunc(mc)

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

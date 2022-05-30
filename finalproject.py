import logging
import time
import datetime as dt
import numpy as np
import os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

#uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E73')
uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E719')
logging.basicConfig(level=logging.ERROR)

# Treshold define
GOTO_TOL = 0.015
TRESHOLD = 300 #treshold pour le contournement de l'obstalce
TRESHOLD_MIN = 150 #va dans la direction oppose à l'obstacle, peut-importe la direction
PAD_TRESHOLD_RISE = 18
PAD_TRESHOLD_FALL = 15
ZSPEEDTRESHOLD = 0.20
SIDE_MARGIN = 0.1
FACE_MARGIN = 0.2
MPAD = 0.08
FRAMEARATER = 4

# Map define
X0 = 1
Y0 = 2
START_ZONE_X = 3.5
RETURN_ZONE_X = 1.5
MAP_SIZE_X = 5
MAP_SIZE_Y = 3

#speed define
EXPLORESPEED = 0.5
LANDINGSPEED = 0.4
GOTOSPEED = 0.7
AVOIDSPEED = 0.3


#Dictonnary
dirToOrderDict = {
  'FRONT': [1,0],
  'RIGHT': [0,-1],
  'BACK' : [-1,0],
  'LEFT' : [0,1]
}
dictAvoid = { #[direcion où aller en fonction de statusManoeuvre; 0 ou 1 pour savoir s'il faut retourner au milieu en x ou en Y ; 0,1,2 ou 3 pour savoir quel capteur (face ou té-co) lire]
  'FRONT': ['LEFT','FRONT','RIGHT', 1, 0, 0, 3],
  'LEFT' : ['BACK','LEFT' ,'FRONT', 0, 1, 1, 0],
  'BACK' : ['LEFT','BACK' ,'RIGHT', 1, 0, 2, 3],
  'RIGHT': ['BACK','RIGHT','FRONT', 0, 1, 3, 0]
}
dictNextState = {
  'LANDINGPADDETECTED' : 'LANDED',
  'STARTINGPADDETECTED' : 'FINISH'
}



class Drone:

    def __init__(self, link_id):
        """ Initialize and run the example with the specified link_id """
        # Initialize cf object
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Initialize log variable
        self.count = 0
        self.step = 0
        self.logs = np.zeros([100000,12])

        #Program status variable
        self.is_connected = True
        self.explorationState = 'START' #[start, landingPadDetected, return, startingPadDetected, finish]

        # Drone variable
        self.heightDrone = 400
        self.x = 0
        self.y = 0
        self.z = 0
        self.front = 0
        self.left = 0
        self.right = 0
        self.back = 0
        self.speedZ = 0
        self.lastZ = 0

        #general usage variable
        self.V_ref = [0, None]
        self.lastV_ref = [0, None]
        self.X0 = X0
        self.Y0 = Y0

        #obstacle avoidance variable
        self.lastPos = [0,0]
        self.goalPos = [0,0]
        self.statusManoeuvre = 0 #[0, 1 ,2] -> obstacle de face, de té-co, derrière
        self.obstacleDetected = None #[None, 'FRONT', 'LEFT', 'RIGHT', 'BACK']
        self.directionChange = None
        # Explore variable
        self.explorePos = [0,0]
        self.xSave = 0
        self.exploreStatus = 'FRONT'

        # Landing variable
        self.landingStatus = 'LAND'
        self.x_pad1=0
        self.y_pad1=0
        self.memory = 0
        self.case = 0
        self.counterStop = 0
        self.saveV_ref = self.V_ref

        #return to start variables
        self.counter = -1
        self.desiredReturnPos = [0,0]
        self.mem = 0
        self.dir = 0

        self.flySearch(link_id)

    def _connected(self, link_id):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self._lg_stab.add_variable('range.zrange', 'uint16_t')
        self._lg_stab.add_variable('range.front', 'uint16_t')
        self._lg_stab.add_variable('range.left', 'uint16_t')
        self._lg_stab.add_variable('range.right', 'uint16_t')
        self._lg_stab.add_variable('range.back', 'uint16_t')
        self._lg_stab.add_variable('stateEstimate.vz', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
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
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(),'logs',filename)
        np.savetxt(filepath, self.logs, delimiter=',')

    def explore(self):
        if(self.x < START_ZONE_X): #première zone
            self.V_ref = [EXPLORESPEED, 'FRONT']
        else: #deuxieme zone : on commence la recherche
            if(self.exploreStatus == 'FRONT'):
                if(self.explorePos[0] > self.x) :
                    self.V_ref = [EXPLORESPEED, 'FRONT']
                else :
                    self.obstacleDetected = None
                    if(self.y < MAP_SIZE_Y/2.):
                        self.exploreStatus = 'LEFT'
                        self.xSave = self.x
                    else :
                        self.exploreStatus = 'RIGHT'
                        self.xSave = self.x
            elif(self.exploreStatus == 'LEFT'):
                if(self.y < MAP_SIZE_Y) :
                    self.V_ref = [EXPLORESPEED, 'LEFT']
                else :
                    self.exploreStatus = 'FRONT'
                    self.explorePos = [self.xSave + 0.2, self.y]
            elif(self.exploreStatus == 'RIGHT'):
                if(self.y > 0) :
                    self.V_ref = [EXPLORESPEED, 'RIGHT']
                else :
                    self.exploreStatus = 'FRONT'
                    self.explorePos = [self.xSave + 0.2, self.y]
            else :
                print('wrong state machine : explore')

    def detectPadBorder(self):
        if((self.speedZ > ZSPEEDTRESHOLD) and (self.obstacleDetected is None) and (self.directionChange is None)):# and ((self.lastZ-self.z) > 0.05)): #On détecte un Rise -> on monte sur le PAD
        #if((self.lastZ - self.z) > PAD_TRESHOLD_RISE):
            if(self.explorationState == 'START' and self.x > START_ZONE_X) :
                # print('PAD DETECTED')
                self.explorationState = 'LANDINGPADDETECTED'
                self.landingStatus = 'FIRST_BORDER'
            elif(self.explorationState == 'RETURN' and self.x < RETURN_ZONE_X):
                self.explorationState = 'STARTINGPADDETECTED'
                self.landingStatus = 'FIRST_BORDER'
            elif(self.landingStatus == 'SEARCH_SECOND_BORDER'):
                self.landingStatus = 'SECOND_BORDER'

        #a voir si on va s'en servir
        self.lastZ = self.z

    def landing(self):
        if(self.case==0):
            if (self.landingStatus == 'FIRST_BORDER' and self.V_ref[1] == 'FRONT'):
                self.x_pad1=self.x
                self.y_pad1=self.y
                self.memory=self.y
                self.case=1
                # print('first_border')
            if (self.landingStatus == 'FIRST_BORDER' and self.V_ref[1] == 'BACK'):
                self.x_pad1=self.x
                self.y_pad1=self.y
                self.memory=self.y
                self.case=3
                # print('first_border')
            if (self.landingStatus == 'FIRST_BORDER' and self.V_ref[1] == 'LEFT'):
                self.y_pad1=self.y
                self.x_pad1=self.x
                self.memory=self.x
                self.case=5
                # print('first_border')
            if (self.landingStatus == 'FIRST_BORDER' and self.V_ref[1] == 'RIGHT'):
                self.y_pad1=self.y
                self.x_pad1=self.x
                self.memory=self.x
                self.case=7
                # print('first_border')
            if (self.landingStatus == 'SECOND_BORDER' and self.V_ref[1] == 'RIGHT'):
                self.y_pad1=self.y
                self.case=2
                # print('second_border')
            if (self.landingStatus == 'SECOND_BORDER' and self.V_ref[1] == 'LEFT'):
                self.y_pad1=self.y
                self.case=4
                # print('second_border')
            if (self.landingStatus == 'SECOND_BORDER' and self.V_ref[1] == 'BACK'):
                self.x_pad1=self.x
                self.case=6
                # print('second_border')
            if (self.landingStatus == 'SECOND_BORDER' and self.V_ref[1] == 'FRONT'):
                self.x_pad1=self.x
                self.case=8
                # print('second_border')
            if(self.landingStatus == 'SEARCH_SECOND_BORDER'):
                print('Stopped', self.counterStop)
                self.counterStop += 1
                if(self.counterStop < 30):
                    self.V_ref = [0,'STOP']
                elif(self.counterStop > 280):
                    if(self.goTo([self.x_pad1,self.y_pad1])):
                        self.V_ref = [0.1,'LAND']
                        self.counterStop = 0
                else:
                    self.V_ref = self.saveV_ref

        if(self.case==1):
            if(self.goTo([self.x_pad1+MPAD, self.memory+0.50])==False and self.landingStatus == 'FIRST_BORDER'):
                print('desired:', self.x_pad1+0.10, self.memory+0.50, 'position:', self.x, self.y)
                
            else:
                self.landingStatus = 'SEARCH_SECOND_BORDER'
                self.case=0
                self.saveV_ref = [LANDINGSPEED,'RIGHT']
                # if(self.directionChange is not None):
                #     self.directionChange+=1
                # if(self.directionChange > FRAMEARATER):
                #     self.directionChange = None
        if(self.case==2):
            if(self.goTo([self.x_pad1+MPAD, self.y_pad1-MPAD])==False and self.landingStatus == 'SECOND_BORDER'):
                print('desired:', self.x_pad1+0.10, self.y_pad1-0.10, 'position:', self.x, self.y)
            else:
                self.case=0
                self.V_ref[1] = 'LAND'
                self.counterStop = 0
        if(self.case==3):
            if(self.goTo([self.x_pad1-MPAD, self.memory-0.50])==False and self.landingStatus == 'FIRST_BORDER'):
                print('desired:', self.x_pad1-0.10, self.memory-0.50, 'position:', self.x, self.y)
            
            else:
                self.landingStatus = 'SEARCH_SECOND_BORDER'
                self.case=0
                self.saveV_ref = [LANDINGSPEED,'LEFT']
                # if(self.directionChange is not None):
                #     self.directionChange+=1
                # if(self.directionChange > FRAMEARATER):
                #     self.directionChange = None
        if(self.case==4):
            if(self.goTo([self.x_pad1-MPAD, self.y_pad1+MPAD])==False and self.landingStatus == 'SECOND_BORDER'):
                print('desired:', self.x_pad1-0.10, self.y_pad1+0.10, 'position:', self.x, self.y)
            else:
                self.case=0
                self.V_ref[1] = 'LAND'
                self.counterStop = 0
        if(self.case==5):
            if(self.goTo([self.memory+0.50, self.y_pad1+MPAD])==False and self.landingStatus == 'FIRST_BORDER'):
                print('desired:', self.memory+0.50, self.y_pad1+0.10, 'position:', self.x, self.y)
                
            else:
                self.landingStatus = 'SEARCH_SECOND_BORDER'
                self.case=0
                self.saveV_ref = [LANDINGSPEED,'BACK']
                # if(self.directionChange is not None):
                #     self.directionChange+=1
                # if(self.directionChange > FRAMEARATER):
                #     self.directionChange = None
        if(self.case==6):
            if(self.goTo([self.x_pad1-MPAD, self.y_pad1+MPAD])==False and self.landingStatus == 'SECOND_BORDER'):
                print('desired:', self.x_pad1-0.10, self.y_pad1+0.10, 'position:', self.x, self.y)
            else:
                self.case=0
                self.V_ref[1] = 'LAND'
                self.counterStop = 0
        if(self.case==7):
            if(self.goTo([self.memory-0.50, self.y_pad1-MPAD])==False and self.landingStatus == 'FIRST_BORDER'):
                print('desired:', self.memory-0.50, self.y_pad1-0.10, 'position:', self.x, self.y)

            else:
                self.landingStatus = 'SEARCH_SECOND_BORDER'
                self.case=0
                self.saveV_ref = [LANDINGSPEED,'FRONT']
                # if(self.directionChange is not None):
                #     self.directionChange+=1
                # if(self.directionChange > FRAMEARATER):
                #     self.directionChange = None
        if(self.case==8):
            if(self.goTo([self.x_pad1+MPAD, self.y_pad1-MPAD])==False and self.landingStatus == 'SECOND_BORDER'):
                print('desired:', self.x_pad1+0.10, self.y_pad1-0.10, 'position:', self.x, self.y)
            else:
                self.case=0
                self.V_ref[1] = 'LAND'
                self.counterStop = 0




    def goTo(self, pos) :
        dist = abs(self.x - pos[0])
        if dist < 0.1:
            speed_applied = 0.3*GOTOSPEED
        elif dist < 0.3:
            speed_applied = 0.6*GOTOSPEED
        else :
            speed_applied = GOTOSPEED

        if(self.x > pos[0] + GOTO_TOL):
            self.V_ref = [speed_applied, 'BACK']
            return False
        elif(self.x < pos[0] - GOTO_TOL):
            self.V_ref = [speed_applied, 'FRONT']
            return False

        dist = abs(self.y - pos[1])
        if dist < 0.1:
            speed_applied = 0.3*GOTOSPEED
        elif dist < 0.3:
            speed_applied = 0.6*GOTOSPEED
        else :
            speed_applied = GOTOSPEED

        if(self.y > pos[1] + GOTO_TOL):
            self.V_ref = [speed_applied, 'RIGHT']
            return False
        elif(self.y < pos[1] - GOTO_TOL):
            self.V_ref = [speed_applied, 'LEFT']
            return False

        return True

    def returnToStart(self):
        # print('counter = ', self.counter)
        # print(self.exploreStatus)

        if (self.counter == -1 and self.x > X0):
            self.V_ref = [EXPLORESPEED, 'BACK']
        if((self.counter == -1) and (X0 -0.03 < self.x < X0 + 0.03)):
            self.counter = 0
            self.mem = self.y
        if (self.counter == 0 and self.mem > Y0 and self.y > Y0-0.2):
            self.V_ref = [EXPLORESPEED, 'RIGHT']
            self.dir = 1
        if (self.counter == 0 and self.mem < Y0 and self.y < Y0 + 0.2):
            self.V_ref = [EXPLORESPEED, 'LEFT']
            self.dir = 2

        if (self.dir == 1):
            if((self.counter == 0) and (Y0 -0.23 < self.y < Y0 - 0.17)):
                self.counter = 1
                self.exploreStatus == 'BACK'
                self.desiredReturnPos = [self.x - 0.2*self.counter, self.y]
        if (self.dir == 2):
            if((self.counter == 0) and (Y0 +0.17 < self.y < Y0 + 0.23)):
                self.counter = 1
                self.exploreStatus == 'BACK'
                self.desiredReturnPos = [self.x - 0.2*self.counter, self.y]
        if (self.counter > 0):
            if(self.exploreStatus == 'BACK'):
                self.V_ref = [EXPLORESPEED, 'BACK']
                if (self.x < self.desiredReturnPos[0]):
                    self.exploreStatus = 'LEFT'
                    self.desiredReturnPos = [self.x, self.y + 0.2*self.counter]

            elif(self.exploreStatus == 'LEFT'):
                self.V_ref = [EXPLORESPEED, 'LEFT']
                if (self.y > self.desiredReturnPos[1]):
                    self.exploreStatus = 'FRONT'
                    self.desiredReturnPos = [self.x + 0.2*self.counter, self.y]
                    self.counter += 1

            elif(self.exploreStatus == 'FRONT'):
                self.V_ref = [EXPLORESPEED, 'FRONT']
                if (self.x > self.desiredReturnPos[0]):
                    self.desiredReturnPos = [self.x, self.y - 0.2*self.counter]
                    self.exploreStatus = 'RIGHT'

            elif(self.exploreStatus == 'RIGHT'):
                self.V_ref = [EXPLORESPEED, 'RIGHT']
                if (self.y < self.desiredReturnPos[1]):
                    self.desiredReturnPos = [self.x - 0.2*self.counter, self.y]
                    self.exploreStatus = 'BACK'
                    self.counter += 1

            else :
                print('wrong state machine : return to start')


    def obstacleDodge_forward(self):
        # print('obstacle dodge forward')
        # print(self.statusManoeuvre)

        #sécurité : si on a mal éviter l'obstacle
        if(self.front < TRESHOLD):
            self.statusManoeuvre = 'FACE'
        if(self.right < TRESHOLD):
            self.statusManoeuvre = 'SIDE'

        #début de l'algo
        if(self.statusManoeuvre == 'FACE'): #obstacle de Face
            if(self.front < TRESHOLD):
                self.V_ref = [AVOIDSPEED,'LEFT']
            else :
                self.goalPos = [self.x, self.y + SIDE_MARGIN]
                self.statusManoeuvre = 'AFTER_FACE'
        elif(self.statusManoeuvre == 'AFTER_FACE'): #on se décale un peu plus
            if(self.goalPos[1] > self.y):
                self.V_ref = [AVOIDSPEED,'LEFT']
            else:
                self.statusManoeuvre = 'SIDE'
                self.goalPos = [self.x + FACE_MARGIN, self.y]
        elif(self.statusManoeuvre == 'SIDE'): #obstacle de coté
            if(self.right < TRESHOLD or (self.x < self.goalPos[0] and self.front > TRESHOLD)):
                self.V_ref = [AVOIDSPEED, 'FRONT']
            else:
                self.goalPos = [self.x + SIDE_MARGIN, self.y]
                self.statusManoeuvre = 'AFTER_SIDE'
        elif(self.statusManoeuvre == 'AFTER_SIDE'):
            if(self.goalPos[0] > self.x):
                self.V_ref = [AVOIDSPEED,'FRONT']
            else:
                self.statusManoeuvre = 'GO_BACK_MIDDLE'
                self.goalPos = [self.x, self.lastPos[1]]
        elif(self.statusManoeuvre == 'GO_BACK_MIDDLE'):
            if(self.goalPos[1] < self.y):
                self.V_ref = [AVOIDSPEED, 'RIGHT']
            else :
                self.obstacleDetected = None
                self.statusManoeuvre = 'FACE'

    def obstacleDodge_left(self):
        # print('obstacle dodge left')
        # print(self.statusManoeuvre)

        #sécurité : si on a mal éviter l'obstacle
        if(self.left < TRESHOLD):
            self.statusManoeuvre = 'FACE'
        if(self.front < TRESHOLD):
            self.statusManoeuvre = 'SIDE'

        #début de l'algo
        if(self.statusManoeuvre == 'FACE'): #obstacle de Face
            if(self.left < TRESHOLD):
                self.V_ref = [AVOIDSPEED,'BACK']
            else :
                self.goalPos = [self.x - SIDE_MARGIN, self.y]
                self.statusManoeuvre = 'AFTER_FACE'
        elif(self.statusManoeuvre == 'AFTER_FACE'): #on se décale un peu plus
            if(self.goalPos[0] < self.x):
                self.V_ref = [AVOIDSPEED,'BACK']
            else:
                self.statusManoeuvre = 'SIDE'
                self.goalPos = [self.x, self.y + FACE_MARGIN]
        elif(self.statusManoeuvre == 'SIDE'): #obstacle de coté
            if(self.front < TRESHOLD or (self.y < self.goalPos[1] and self.left > TRESHOLD)):
                self.V_ref = [AVOIDSPEED, 'LEFT']
            else:
                self.goalPos = [self.x, self.y + SIDE_MARGIN]
                self.statusManoeuvre = 'AFTER_SIDE'
        elif(self.statusManoeuvre == 'AFTER_SIDE'):
            if(self.goalPos[1] > self.y):
                self.V_ref = [AVOIDSPEED,'LEFT']
            else:
                self.statusManoeuvre = 'GO_BACK_MIDDLE'
                self.goalPos = [self.lastPos[0], self.y]
        elif(self.statusManoeuvre == 'GO_BACK_MIDDLE'):
            if(self.goalPos[0] > self.x):
                self.V_ref = [AVOIDSPEED, 'FRONT']
            else :
                self.obstacleDetected = None
                self.statusManoeuvre = 'FACE'

    def obstacleDodge_right(self):
        # print('obstacle dodge right')
        # print(self.statusManoeuvre)

        #sécurité : si on a mal éviter l'obstacle
        if(self.right < TRESHOLD):
            self.statusManoeuvre = 'FACE'
        if(self.front < TRESHOLD):
            self.statusManoeuvre = 'SIDE'

        #début de l'algo
        if(self.statusManoeuvre == 'FACE'): #obstacle de Face
            if(self.right < TRESHOLD):
                self.V_ref = [AVOIDSPEED,'BACK']
            else :
                self.goalPos = [self.x - SIDE_MARGIN, self.y]
                self.statusManoeuvre = 'AFTER_FACE'
        elif(self.statusManoeuvre == 'AFTER_FACE'): #on se décale un peu plus
            if(self.goalPos[0] < self.x):
                self.V_ref = [AVOIDSPEED,'BACK']
            else:
                self.statusManoeuvre = 'SIDE'
                self.goalPos = [self.x, self.y - FACE_MARGIN]
        elif(self.statusManoeuvre == 'SIDE'): #obstacle de coté
            if(self.front < TRESHOLD or (self.y > self.goalPos[1] and self.right > TRESHOLD)):
                self.V_ref = [AVOIDSPEED, 'RIGHT']
            else:
                self.goalPos = [self.x, self.y - SIDE_MARGIN]
                self.statusManoeuvre = 'AFTER_SIDE'
        elif(self.statusManoeuvre == 'AFTER_SIDE'):
            if(self.goalPos[1] < self.y):
                self.V_ref = [AVOIDSPEED,'RIGHT']
            else:
                self.statusManoeuvre = 'GO_BACK_MIDDLE'
                self.goalPos = [self.lastPos[0], self.y]
        elif(self.statusManoeuvre == 'GO_BACK_MIDDLE'):
            if(self.goalPos[0] > self.x):
                self.V_ref = [AVOIDSPEED, 'FRONT']
            else :
                self.obstacleDetected = None
                self.statusManoeuvre = 'FACE'

    def obstacleDodge_back(self):
        print('obstacle dodge back')
        print(self.statusManoeuvre)

        #sécurité : si on a mal éviter l'obstacle
        if(self.back < TRESHOLD):
            self.statusManoeuvre = 'FACE'
        if(self.right < TRESHOLD):
            self.statusManoeuvre = 'SIDE'

        #début de l'algo
        if(self.statusManoeuvre == 'FACE'): #obstacle de Face
            if(self.back < TRESHOLD):
                self.V_ref = [AVOIDSPEED,'LEFT']
            else :
                self.goalPos = [self.x, self.y + SIDE_MARGIN]
                self.statusManoeuvre = 'AFTER_FACE'
        elif(self.statusManoeuvre == 'AFTER_FACE'): #on se décale un peu plus
            if(self.goalPos[1] > self.y):
                self.V_ref = [AVOIDSPEED,'LEFT']
            else:
                self.statusManoeuvre = 'SIDE'
                self.goalPos = [self.x - FACE_MARGIN, self.y]
        elif(self.statusManoeuvre == 'SIDE'): #obstacle de coté
            if(self.right < TRESHOLD or (self.x > self.goalPos[0] and self.front > TRESHOLD)):
                self.V_ref = [AVOIDSPEED, 'BACK']
            else:
                self.goalPos = [self.x - SIDE_MARGIN, self.y]
                self.statusManoeuvre = 'AFTER_SIDE'
        elif(self.statusManoeuvre == 'AFTER_SIDE'):
            if(self.goalPos[0] < self.x):
                self.V_ref = [AVOIDSPEED,'BACK']
            else:
                self.statusManoeuvre = 'GO_BACK_MIDDLE'
                self.goalPos = [self.x, self.lastPos[1]]
        elif(self.statusManoeuvre == 'GO_BACK_MIDDLE'):
            if(self.goalPos[1] < self.y):
                self.V_ref = [AVOIDSPEED, 'RIGHT']
            else :
                self.obstacleDetected = None
                self.statusManoeuvre = 'FACE'




    def obstacleDodge(self):
        """Permet de contourner l'obstacle
        1. on se décale tant qu'on détécte l'obstacle de face, puis encore de 7cm
        2. on avance de 7cm, puis tant qu'on détécte l'obstacle sur le té-co, puis encore de 7cm
        3. on se recentre (uniquement le long de la coordonée d'intéret)"""
        print('Obstacle dodging')
        capteurFace = [self.front, self.left, self.back, self.right][dictAvoid[self.obstacleDetected][5]]
        capteurSide = [self.front, self.left, self.back, self.right][dictAvoid[self.obstacleDetected][6]]
        if(self.statusManoeuvre == 0): # Manoeuvre pour éviter l'osbtacle qui est de face
            if(capteurFace < TRESHOLD): # Tant qu'on détecte l'obstacle en face on avance sur le té-co et on reset l'objectif 7cm plus loin (sur le té-co)
                self.V_ref = [AVOIDSPEED, dictAvoid[self.obstacleDetected][self.statusManoeuvre]]
                self.goalPos = [self.x + 0.2*dirToOrderDict[dictAvoid[self.obstacleDetected][self.statusManoeuvre]][0], self.y + 0.2*dirToOrderDict[dictAvoid[self.obstacleDetected][self.statusManoeuvre]][1]] # tant qu'il detecte il fixe son obj 7cm de plus du côté où il est entrain d'aller
            else: # On détecte plus l'objectif sur en face, alors on move à l'objectif qui est 7cm plus loin
                if(self.goTo(self.goalPos)): #On a move de 7cm sur le té-co, alors on fixe l'objectif 7cm devant et on passe au point suivant
                    self.statusManoeuvre = 1
                    self.goalPos = [self.x + 0.2*dirToOrderDict[dictAvoid[self.obstacleDetected][self.statusManoeuvre]][0], self.y + 0.2*dirToOrderDict[dictAvoid[self.obstacleDetected][self.statusManoeuvre]][1]] #7cm de plus du côté où il VA d'aller
                    print("First border avoided, actual pos :", [self.x, self.y], 'desired pos :', self.goalPos)
        elif(self.statusManoeuvre == 1): #l'obstacle sur le téco
            if(capteurFace < TRESHOLD):#on redétècte l'obstacle en face -> retour au début, on s'est pas assez décalé
                self.statusManoeuvre = 0
            elif(capteurSide < TRESHOLD): # Tant qu'on détecte l'obstacle sur le té-co, on avance et on reset l'objectif 7cm plus loin (devant)
                self.V_ref = [AVOIDSPEED, dictAvoid[self.obstacleDetected][self.statusManoeuvre]] #ligne 'direction de l'obstacle' dans le dictionnaire, colone/indice 'statusManoeuvre'
                self.goalPos = [self.x + 0.2*dirToOrderDict[dictAvoid[self.obstacleDetected][self.statusManoeuvre]][0], self.y + 0.2*dirToOrderDict[dictAvoid[self.obstacleDetected][self.statusManoeuvre]][1]] # tant qu'il detecte il fixe son obj 7cm de plus du côté où il est entrain d'aller
            else: #on ne détecte PAS ENCORE ou PLUS l'objectif sur le té-co, donc on move de 7cm en avant (une fois au début, une fois à la fin).
                if(self.goTo(self.goalPos)): # on a move de 7cm et donc dépassé l'obstacle, on passe au point suivant et l'objectif au point de départ de l'obstalce avoidance sera fixé après (pour permettre de pas être coincé par le goTo en refreshant l'objectif)
                    self.statusManoeuvre = 2
        elif(self.statusManoeuvre == 2): # manoeuvre finale, on se redecale pour atteindre la position de départ.
            if(capteurSide < TRESHOLD): # On redecte l'obstacle sur le té-co, retour a l'étape précédante, on a pas assez avancé
                self.statusManoeuvre = 1
            self.goalPos = [ self.x*dictAvoid[self.obstacleDetected][3] + self.lastPos[0]*dictAvoid[self.obstacleDetected][4], self.y*dictAvoid[self.obstacleDetected][4] + self.lastPos[1]*dictAvoid[self.obstacleDetected][3]] #position pour le faire revenir là où il avait commencé à éviter l'obstacle, on refresh parce que la position est importante seulement dans une des deux coordonées, il doit pouvoir avancer (grâce à l'évitement des obtsalces vraiment trop près) si l'obstalce est de biais
            if(self.goTo(self.goalPos)): #On a atteind la position du début et dépassé l'obstacle. Fin de l'obstacle avoidance
                self.statusManoeuvre = 0
                self.obstacleDetected = None
        #print('obstacle :',self.obstacleDetected,', statusManoeuvre :', self.statusManoeuvre,'   , direction :',self.V_ref[1])

    def obstacleAvoidance(self,mc):
        """ Fonction qui applique la vitesse au drone en fonction de la référence et des obstacles
        1. Vérifie s'il y a un obstalce sur le chemin -> si oui, lance obstacleDodge pour l'éviter (garde en mémoire l'état)
        2. s'éloigne des obstacles TROP proches
        3. applique la vitesse de référence au drone """
        if(self.directionChange is not None):
            # print('Direction change : ', self.directionChange)
            self.directionChange += 1
            if(self.directionChange > 50):
                self.directionChange = None


        if(self.V_ref[1] == 'LAND'):
            [self.X0, self.Y0] = [self.x, self.y]
            mc.land()
            self.explorationState = dictNextState[self.explorationState]
            return

        #détecte si on "découvre" un obstacle de face
        if(self.obstacleDetected is None):
            if(self.front < TRESHOLD and self.V_ref[1] == 'FRONT') :
                self.obstacleDetected = 'FRONT'
            elif(self.left < TRESHOLD and self.V_ref[1] == 'LEFT') :
                self.obstacleDetected = 'LEFT'
            elif(self.right < TRESHOLD and self.V_ref[1] == 'RIGHT') :
                self.obstacleDetected = 'RIGHT'
            elif(self.back < TRESHOLD and self.V_ref[1] == 'BACK') :
                self.obstacleDetected = 'BACK'
            if(self.obstacleDetected is not None):
                self.lastPos = [self.x, self.y]
            if((self.x > RETURN_ZONE_X) and (self.statusManoeuvre == 'GO_BACK_MIDDLE') and (self.explorationState == 'RETURN')):
                self.obstacleDetected = None
                self.statusManoeuvre = 'FACE'

        # gère l'évitement de l'obstacle de face
        if(self.obstacleDetected is not None) :
            if(self.obstacleDetected == 'FRONT'):
                self.obstacleDodge_forward()
            if(self.obstacleDetected == 'LEFT'):
                self.obstacleDodge_left()
            if(self.obstacleDetected == 'RIGHT'):
                self.obstacleDodge_right()
            if(self.obstacleDetected == 'BACK'):
                self.obstacleDodge_back()

        #permet d'éviter les obstalces TROP proche, dans toutes les directions, actif tout le temps
        if(self.front < TRESHOLD_MIN):
            self.V_ref = [AVOIDSPEED, 'BACK']
            self.directionChange = 1
        elif(self.left < TRESHOLD_MIN):
            self.V_ref = [AVOIDSPEED, 'RIGHT']
            self.directionChange = 1
        elif(self.right < TRESHOLD_MIN):
            self.V_ref = [AVOIDSPEED, 'LEFT']
            self.directionChange = 1
        elif(self.back < TRESHOLD_MIN):
            self.V_ref = [AVOIDSPEED, 'FRONT']
            self.directionChange = 1

        # Set la vitesse
        if(self.lastV_ref != self.V_ref):
            if(self.V_ref[1] == 'FRONT') :
                mc.start_forward(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'LEFT') :
                mc.start_left(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'RIGHT') :
                mc.start_right(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'BACK') :
                mc.start_back(velocity=self.V_ref[0])
            elif(self.V_ref[1] == 'STOP'):
                mc.stop()
            else:
                print('wrong state machine : obstacle avoidance')
        self.lastV_ref = self.V_ref
        #print('obstacle :',self.obstacleDetected,', statusManoeuvre :', self.statusManoeuvre,'   , direction :',self.V_ref[1], ',   position :',[self.x, self.y])

    def updateSensorValue(self):
        data = self.logs[self.count][:]
        self.x = data[0] + self.X0 #add offset
        self.y = data[1] + self.Y0
        self.z = data[2]
        self.front = data[3]
        self.left = data[4]
        self.right = data[5]
        self.back = data[6]
        self.speedZ = data[7]

    def flySearch(self, id):
        """ algorith de navigation"""
        with SyncCrazyflie(id, cf=self._cf) as scf: # Sync with drone
            with MotionCommander(scf, default_height=(self.heightDrone/1000.)) as mc:
                time.sleep(1)

                #while(self.z > (self.heightDrone + 5) or self.z < (self.heightDrone - 5)) :
                    #self.updateSensorValue()
                    #time.sleep(0.01)
                    #print('Stabilizing height, z :',self.z)

                while(self.explorationState != 'FINISH'):
                    startTime = time.time()

                    self.updateSensorValue()
                    self.detectPadBorder()

                    if(self.explorationState == 'START'):
                        self.explore()
                    elif(self.explorationState == 'LANDINGPADDETECTED'):
                        self.landing()
                    elif(self.explorationState == 'LANDED'):
                        time.sleep(1)
                        mc.take_off(height=self.heightDrone/1000.)
                        self.V_ref[1] = 'STOP'
                        self.explorationState = 'RETURN'

                    elif(self.explorationState == 'RETURN'):
                        self.returnToStart()
                    elif(self.explorationState == 'STARTINGPADDETECTED'):
                        self.landing()
                    elif(self.explorationState == 'FINISH'):
                        self._disconnected
                    else :
                        print("Error : wrong state machine")

                    print(self.explorationState,', z : %2.2f' % self.z,'    Z speed : %2.2f' % self.speedZ,'    Direction : ', self.V_ref[1], '[x y] : [%2.2f, %2.2f]'% (self.x, self.y))

                    self.obstacleAvoidance(mc)
                    nowTime = time.time()
                    if((nowTime - startTime) >= 0.01):
                        print('Too long loop : asynchronous')
                    else :
                        time.sleep(0.01 - (nowTime - startTime))

        self._disconnected

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    drone = Drone(uri)
    while drone.is_connected:
        time.sleep(1)

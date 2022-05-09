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

# [start, landingPadDetected, return, startingPadDetected, finish]

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
        # self.explorationState = start
        self.obstacleDetected = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.front = 0
        self.left = 0
        self.right = 0
        self.back = 0
        self.x_zone=0

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

    # def explore(self, x0, y0, mc):
    #     while(self.logs[self.count][0] < (1.0-x0)):
    #         mc.start_forward(velocity=0.2)
    #         print('go forward 1m')
    #         print('x = ', self.x)
    #         print('x0 =', x0)
    #         self.x_zone = self.logs[self.count][0]
    #         # mc.start_left(velocity=0.2)
    #         print('-y+y0 =',-self.logs[self.count][1]+y0 )
    #         print('self x',self.logs[self.count][0] , 'x_zone + 0.3', self.x_zone+0.3)
    #     while((-self.logs[self.count][1]+y0>0) and (self.step==0)):
    #         mc.start_left(velocity=0.2)
    #         #self.obstacleAvoidance_left(mc)
    #         print('go left 1')
    #     self.step=1
    #     print('step1')
    #     while((self.logs[self.count][0] < self.x_zone+0.3) and (self.step==1)):
    #         mc.start_forward(velocity=0.2)
    #         print('if step 1')
    #     self.step=2
    #     print('step2')
    #     while((-self.logs[self.count][0]+y0 < 0.5) and (self.step==2)):
    #         mc.start_right(velocity=0.2)
    #         #self.obstacleAvoidance_right(mc)
    #         print('if step 2')
    #     self.step=3
    #     print('step3')
            # if(self.x < self.x_zone+0.6 and self.step==3):
            #     mc.start_forward(velocity=0.2)
            #     print('if step 3')
            # elif(self.x >= self.x_zone+0.6 and self.step==3):
            #     self.step=4
            #     print('step4')
            # if(-self.y+y0>0 and self.step==4):
            #     mc.start_left(velocity=0.2)
            #     #self.obstacleAvoidance_left(mc)
            #     print('if step 4')
            # elif(-self.y+y0<=0 and self.step==4):
            #     self.step=5
            #     print('step5')
            # if(self.x < self.x_zone+0.9 and self.step==5):
            #     mc.start_forward(velocity=0.2)
            #     print('if step 5')
            # elif(self.x >= self.x_zone+0.9 and self.step==5):
            #     self.step=6
            #     print('step6')
            # if(-self.y+y0 < 0.5 and self.step==6):
            #     mc.start_right(velocity=0.2)
            #     #self.obstacleAvoidance_right(mc)
            #     print('if step 6')
            # elif(-self.y+y0 >= 0.5 and self.step==6):
            #     self.step=7
            #     print('step7')
            # if(self.x < self.x_zone+1.2 and self.step==7):
            #     mc.start_forward(velocity=0.2)
            #     print('if step 7')
            #     self.step=8
            #     print('step8')
            # if(-self.y+y0>0 and self.step==8):
            #     mc.start_left(velocity=0.2)
            #     #self.obstacleAvoidance_left(mc)
            #     print('if step 8')
            # elif(-self.y+y0<=0 and self.step==8):
            #     mc.land()
            #     print('land')

    def explore(self, x0, y0, mc):
        if(self.x < (1-x0)):
            mc.start_forward(velocity=0.2)
            self.x_zone = self.x
        else:
            if((-self.y+y0>0) and (self.step==0)):
                mc.start_left(velocity=0.2)
                if self.z < 585:
                    self.direction=LEFT
                    self.landing(mc)
                if(self.logs[self.count][4] < 300):
                    self.obstacleAvoidance_left(mc)
            elif((-self.y+y0<=0) and (self.step==0)):
                self.step=1
            if((self.x < self.x_zone+0.3) and (self.step==1)):
                mc.move_distance(distance_x_m= 0.3, distance_y_m = 0, distance_z_m = 0,velocity=0.2)
            elif((self.x >= self.x_zone+0.3) and (self.step==1)):
                self.step=2
            if((-self.y+y0 < 0.4) and (self.step==2)):
                mc.start_right(velocity=0.2)
                if self.z < 585:
                    self.direction=RIGHT
                    self.landing(mc)
                if(self.logs[self.count][5] < 300):
                    self.obstacleAvoidance_right(mc)
            elif((-self.y+y0 >= 0.4) and (self.step==2)):
                self.step=3
            if(self.x < self.x_zone+0.6 and self.step==3):  
                mc.move_distance(distance_x_m= 0.3, distance_y_m = 0, distance_z_m = 0,velocity=0.2)                
            elif(self.x >= self.x_zone+0.6 and self.step==3):
                self.step=4
            if(-self.y+y0>0 and self.step==4):
                mc.start_left(velocity=0.2)
                if self.z < 585:
                    self.direction=LEFT
                    self.landing(mc)
                if(self.logs[self.count][4] < 300):
                    self.obstacleAvoidance_left(mc)
            elif(-self.y+y0<=0 and self.step==4):
                self.step=5
            if(self.x < self.x_zone+0.9 and self.step==5):
                mc.move_distance(distance_x_m= 0.3, distance_y_m = 0, distance_z_m = 0,velocity=0.2)
            elif(self.x >= self.x_zone+0.9 and self.step==5):
                self.step=6
            if(-self.y+y0 < 0.4 and self.step==6):
                mc.start_right(velocity=0.2)
                if self.z < 585:
                    self.direction=RIGHT
                    self.landing(mc)
                if(self.logs[self.count][5] < 300):
                    self.obstacleAvoidance_right(mc)
            elif(-self.y+y0 >= 0.4 and self.step==6):
                self.step=7
            if(self.x < self.x_zone+1.2 and self.step==7):
                mc.move_distance(distance_x_m= 0.3, distance_y_m = 0, distance_z_m = 0,velocity=0.2)
            if(self.x > self.x_zone+1.2 and self.step==7):
                self.step=8
            if(-self.y+y0>0 and self.step==8):
                mc.start_left(velocity=0.2)
                if self.z < 585:
                    self.direction=LEFT
                    self.landing(mc)
                if(self.logs[self.count][4] < 300):
                    self.obstacleAvoidance_left(mc)
            elif(-self.y+y0<=0 and self.step==8):
                mc.land()
        

    def landing(self, mc):      #Fonction called when landing pad is detected below the drone
        
        print('la fct de balou elle est éclaté')
        if self.z > HEIGHT_DRONE*1.1 and self.landing_state == 0:
            mc.stop()
            self.landing_state = 1

        if self.landing_state == 1:

            if self.direction == FORWARD:                       #Go back to center of landing pad, depending on where the drone came from and update the direction variable
                mc.back(0.15, velocity = 0.5)      #Check when the drone start going backaward, what's the distance to be in center of the landing pad (I assumed 15cm)
                mc.start_right(velocity = 0.5)
                self.direction = RIGHT
            elif self.direction == BACKWARD:
                mc.forward(0.15, velocity = 0.5)
                mc.start_right(velocity = 0.5)
                self.direction = RIGHT
            elif self.direction == RIGHT:
                mc.left(0.15, velocity = 0.5)
                mc.start_forward(velocity = 0.5)
                self.direction = FORWARD
            elif self.direction == LEFT:
                mc.right(0.15, velocity = 0.5)
                mc.start_forward(velocity = 0.5)
                self.direction = FORWARD

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
        print('AVOIDANCE1')
        y1 = self.logs[self.count][1]
        while (self.logs[self.count][3] < 300):
            mc.start_left(velocity=0.2)
            print('AVOIDANCE1.0')
            print(self.logs[self.count][3])
        mc.left(distance_m = 0.07, velocity=0.2)
        mc.forward(distance_m= 0.3,velocity=0.2)
        while (self.logs[self.count][5] < 500):
            print(self.logs[self.count][4])
            mc.start_forward(velocity=0.2)
        mc.forward(distance_m= 0.1,velocity=0.2)
        mc.right(distance_m = -y1+self.logs[self.count][1],velocity=0.2)

    def obstacleAvoidance_left(self, mc): 
        x1 = self.logs[self.count][0]
        while(self.logs[self.count][4] < 300):
            print('AVOIDANCE_left')
            print('sensor = ', self.logs[self.count][4] )
            mc.start_forward(velocity=0.2)
        mc.move_distance(distance_x_m= 0.1, distance_y_m = 0, distance_z_m = 0,velocity=0.2)
        mc.move_distance(distance_x_m= 0, distance_y_m = 0.3, distance_z_m = 0,velocity=0.2)
        while(self.logs[self.count][6] < 300):
            print('AVOIDANCE2.1')
            mc.start_left(velocity=0.2)
        mc.move_distance(distance_x_m= 0, distance_y_m = 0.1, distance_z_m = 0,velocity=0.2)
        mc.move_distance(distance_x_m= x1-self.logs[self.count][0], distance_y_m = 0, distance_z_m = 0,velocity=0.2)

    def obstacleAvoidance_right(self,mc): 
        print('AVOIDANCE3')
 
        x1 = self.logs[self.count][0]
        while (self.logs[self.count][5] < 300):
            print(self.logs[self.count][5])
            mc.start_forward(velocity=0.2)
            print('AVOIDANCE3.0')
        mc.forward(distance_m= 0.1, velocity=0.2)
        mc.right(distance_m = 0.3, velocity=0.2)
        while (self.logs[self.count][6] < 400):
            mc.start_right(velocity=0.2)
            print('AVOIDANCE3.1')
            print(self.logs[self.count][6])
        print('xm = ', x1-self.logs[self.count][0])
        print('x1 = ', x1)
        print('x_pos', self.logs[self.count][0])
        mc.right(distance_m = 0.1, velocity=0.2)
        mc.back(distance_m= -x1+self.logs[self.count][0], velocity=0.2)

    def obstacleAvoidance():
        return None
    def test(self, mc):
        mc.start_forward(0.2)
        if(self.logs[self.count][3] < 300):
            self.obstacleAvoidance_forward(mc)


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
                    # if self.z < 280:
                    #     print('STOP')
                    #     mc.land()        
                    #     # self._disconnected


                    # print('z =', self.z)
                    # if(self.explorationState == start):
                    self.explore(x0,y0,mc)
                    #self.test(mc)
                    #print('step:', self.step)
                    #print(self.logs[self.count][6])
                    # elif(self.explorationState == landingPadDetected):
                    #     self.landing()
                    # elif(self.explorationState == return):
                    #     self.returnToStart()
                    # elif(self.explorationState == startingPadDetected):
                    #     self.landing()
                    # elif(self.explorationState == finish):
                    #     self._disconnected
                    # else :
                    #     print("Error : wrong state machine")

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

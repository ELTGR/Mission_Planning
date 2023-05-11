#!/usr/bin/env python

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
import numpy as np
import return_on_target 
from datetime import datetime
from sensor_msgs.msg import LaserScan,CompressedImage
import rospy


class Bridge(object):
    """ MAVLink bridge

    Attributes:
        conn (TYPE): MAVLink connection
        data (dict): Deal with all data
    """
    def __init__(self, device='udpin:192.168.2.1:14560', baudrate=115200):
        """
        Args:
            device (str, optional): Input device
                https://ardupilot.github.io/MAVProxy/html/getting_started/starting.html#master
            baudrate (int, optional): Baudrate for serial communication
        """
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        

        self.conn.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.conn.target_system, self.conn.target_component))

        self.data = {}
        self.current_pose = [0,0,0,0,0,0]
        self.mission_point = 0
        self.mission_point_sent = False
        self.init_evit = False
        self.x_evit = np.array([])
        self.scan = LaserScan()
        self.scan_subscriber= rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.video_subscriber= rospy.Subscriber("/video/compressed", LaserScan, self.scan_callback, queue_size=1)
        self.video_subscriber= rospy.Subscriber("/BlueRov2/video/compressed", CompressedImage, self.video_callback , queue_size=1)
        self.video=np.array([])
        self.ok_pose = False
        self.obj_1_pose = False
        self.obj_2_pose = False
        self.obj_3_pose = False
        self.obj_4_pose = False
        self.temps_1=0
        self.temps_2=0
        self.temps_3=0
        self.temps_4=0

        #flag pour savoir quand le parcour et terminer
        self.flag_end=False
        #flag d'ajoue de point 
        self.flag_add_point=True
        #counter_start_stop  pour l'enregistrement camera des points dinteret 
        self.counter_start_stop  = 0
        self.last_pos=0

    def scan_callback(self, data):
        self.scan = data

    def get_data(self):
        """ Return data

        Returns:
            TYPE: Dict
        """
        return self.data
    
    def video_callback(self, data): 
        self.video=data
        #print(self.video)
    def get_all_msgs(self):
        """ Return all mavlink messages

        Returns:
            TYPE: dict
        """
        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        """ Update data dict
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    def print_data(self):
        """ Debug function, print data dict
        """
        print(self.data)

    def set_mode(self, mode):
        """ Set ROV mode
            http://ardupilot.org/copter/docs/flight-modes.html

        Args:
            mode (str): MMAVLink mode

        Returns:
            TYPE: Description
        """
        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)

    def decode_mode(self, base_mode, custom_mode):
        """ Decode mode from heartbeat
            http://mavlink.org/messages/common#heartbeat

        Args:
            base_mode (TYPE): System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            custom_mode (TYPE): A bitfield for use for autopilot-specific flags.

        Returns:
            [str, bool]: Type mode string, arm state
        """
        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_guided_mode(self):
        """ Set guided mode
        """
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)

    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        """ Function to abstract long commands

        Args:
            command (mavlink command): Command
            params (list, optional): param1, param2, ..., param7
            confirmation (int, optional): Confirmation value
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            command,                                # mavlink command
            confirmation,                           # confirmation
            params[0],                              # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    # def set_position_target_local_ned(self, param=[]):
    #     """ Create a SET_POSITION_TARGET_LOCAL_NED message
    #         http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED

    #     Args:
    #         param (list, optional): param1, param2, ..., param11
    #     """
    #     if len(param) != 11:
    #         print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

    #     # Set mask
    #     mask = 0b0000000111111111
    #     # mask = 0b0000000111000000
    #     for i, value in enumerate(param):
    #         if value is not None:
    #             mask -= 1<<i
    #         else:
    #             param[i] = 0.0

    #     #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
    #     self.conn.mav.set_position_target_local_ned_send(
    #         0,                                              # system time in milliseconds
    #         self.conn.target_system,                        # target system
    #         self.conn.target_component,                     # target component
    #         mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
    #         mask,                                           # mask
    #         param[0], param[1], param[2],                   # position x,y,z
    #         param[3], param[4], param[5],                   # velocity x,y,z
    #         param[6], param[7], param[8],                   # accel x,y,z
    #         param[9], param[10])                            # yaw, yaw rate


    def set_position_target_local_ned(self, param=[]):
        """ Create a SET_POSITION_TARGET_LOCAL_NED message
            http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED

        Args:
            param (list, optional): param1, param2, ..., param11
        """
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

        

        while not self.is_armed():
            self.conn.arducopter_arm()

        self.conn.set_mode('GUIDED')
        # Set mask
        # mask = 0b0000000111111111
        mask = 0b10011111000
        # for i, value in enumerate(param):
        #     if value is not None:
        #         mask -= 1<<i
        #     else:
        #         param[i] = 0.0


        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.conn.target_system, self.conn.target_component,
                           mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(mask),
                            param[0], param[1], param[2],                   # position x,y,z
                            param[3], param[4], param[5],                   # velocity x,y,z
                            param[6], param[7], param[8],                   # accel x,y,z
                            param[9], param[10]))                           # yaw, yaw rate


    def set_attitude_target(self, param=[]):
        """ Create a SET_ATTITUDE_TARGET message
            http://mavlink.org/messages/common#SET_ATTITUDE_TARGET

        Args:
            param (list, optional): param1, param2, ..., param7
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.conn.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust

    def set_servo_pwm(self, id, pwm=1500):
        """ Set servo pwm

        Args:
            id (int): Servo id
            pwm (int, optional): pwm value 1100-2000
        """

        #http://mavlink.org/messages/common#MAV_CMD_DO_SET_SERVO
        # servo id
        # pwm 1000-2000
        mavutil.mavfile.set_servo(self.conn, id, pwm)

    def set_rc_channel_pwm(self, id, pwm=1500):
        """ Set RC channel pwm value

        Args:
            id (TYPE): Channel id
            pwm (int, optional): Channel pwm value 1100-2000
        """
        rc_channel_values = [65535 for _ in range(8)] #8 for mavlink1
        rc_channel_values[id] = pwm
        #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.
    
    def set_manual_control(self,joy_list=[0]*4, buttons_list=[0]*16):
        """ Set a MANUAL_CONTROL message for dealing with more control with ArduSub
        for now it is just to deal with lights under test...
        """
        x,y,z,r = 0,0,0,0#32767,32767,32767,32767
        b = 0
        for i in range(len(buttons_list)):
            b = b | (buttons_list[i]<<i)
        print("MANUAL_CONTROL_SEND : x : {}, y : {}, z : {}, r : {}, b : {}".format(x,y,z,r,b))
        #https://mavlink.io/en/messages/common.html MANUAL_CONTROL ( #69 )
        self.conn.mav.manual_control_send(
               self.conn.target_system,
                x,
                y,
                z,
                r,
                b)


    def arm_throttle(self, arm_throttle):
        """ Arm throttle

        Args:
            arm_throttle (bool): Arm state
        """
        if arm_throttle:
            self.conn.arducopter_arm()
        else:
            #http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM
            # param1 (0 to indicate disarm)
            # Reserved (all remaining params)
            self.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0, 0, 0, 0, 0, 0, 0]
            )



    def set_target_depth(self,depth):

        self.conn.set_mode('ALT_HOLD')

        while not self.is_armed():
            self.conn.arducopter_arm()

        print('Bluerov is armed')

        self.conn.mav.set_position_target_global_int_send(
            0,     
            0, 0,   
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
            0b0000111111111000,
            0,0, depth,
            0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
            0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
            0 , 0 ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

        print('set_position_target_global_int_send')    

    def is_armed(self):
        try:
            return bool(self.conn.wait_heartbeat().base_mode & 0b10000000)
        except:
            return False  


    def set_target_attitude(self, roll, pitch, yaw, control_yaw=True):
        bitmask = (1<<6 | 1<<3)  if control_yaw else 1<<6

        self.conn.mav.set_attitude_target_send(
            0,     
            0, 0,   
            bitmask,
            QuaternionBase([math.radians(roll), math.radians(pitch), math.radians(yaw)]), # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            0, #roll rate
            0, #pitch rate
            0, 0)    # yaw rate, thrust 
    
    def get_bluerov_position_data(self): #Loop comunicazione con QGROUND
        # Get some information !
        
        
        
        if 'LOCAL_POSITION_NED' in self.get_data():              
            
            local_position_data = self.get_data()['LOCAL_POSITION_NED']
            local_oritentation_data = self.get_data()['ATTITUDE']
            #gps_position_data = self.get_data()['GPS_RAW_INT']

            xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
            log_ori_data = [local_oritentation_data[i]  for i in ['yaw']]
            log_pos_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
            self.current_pose[0:3] = [xyz_data[0], xyz_data[1], xyz_data[2]]
            #lat_long_alt_data = [gps_position_data[i]  for i in ['lat', 'lon', 'alt']]
            
            #print(lat_long_alt_data)


            if self.last_pos!=log_pos_data[1] :
                
                fichier = open("log_position.txt", "a")
                fichier.write(str(datetime.now().time()))
                fichier.write(',')
            
                for i in range(0,3):

                    fichier.write(str(log_pos_data[i]))
                    fichier.write(",")

                fichier.write(str(log_ori_data[0]))
                fichier.write(",\n")
                fichier.close()
                self.last_pos=log_pos_data[1] 
                
                
        # waiting for 2 seconds after writing
        # the file
        # else:
        #     print("no local position ned")
        # time.sleep(2)
        # print("Finished background file write to",
        #                                  self.out)  


    def deterministe_scan(self,current_pose):
        obj=[[-6.27,-31.47],
             [-1.57,-25.62],
             [-11.22,-24.74],
             [-3.62,-19.98]
             ]
        
        if abs(current_pose[0] - obj[0][0]) < 0.25 and abs(current_pose[1] - obj[0][1]) < 0.25 and  not(self.obj_1_pose) :
            print("trouver obj1" ) 
            self.obj_1_pose=True
            fichier = open("log_position.txt", "r")
            data=fichier.readlines()[-1]
            fichier.close()
            self.temps_1=data.split(',')[0]
            
        if abs(current_pose[0] - obj[1][0]) < 0.25 and abs(current_pose[1] - obj[1][1]) < 0.25 and  not(self.obj_2_pose) :
            print("trouver obj2" ) 
            self.obj_2_pose=True
            fichier = open("log_position.txt", "r")
            data=fichier.readlines()[-1]
            fichier.close()
            self.temps_2=data.split(',')[0]
            
        if abs(current_pose[0] - obj[2][0]) < 0.25 and abs(current_pose[1] - obj[2][1]) < 0.25 and  not(self.obj_3_pose) :
            print("trouver obj3" ) 
            self.obj_3_pose=True
            fichier = open("log_position.txt", "r")
            data=fichier.readlines()[-1]
            fichier.close()
            self.temps_3=data.split(',')[0]
            
        if abs(current_pose[0] - obj[3][0]) < 0.25 and abs(current_pose[1] - obj[3][1]) < 0.25 and  not(self.obj_4_pose) :
            print("trouver obj4" ) 
            self.obj_4_pose=True
            fichier = open("log_position.txt", "r")
            data=fichier.readlines()[-1]
            fichier.close()
            self.temps_4=data.split(',')[0]
            

    import numpy as np
    from matplotlib import pyplot as plt
    def angle_to_point(self, goal) :
        
        current_pose = self.current_pose
        
        p0 = [round(current_pose[0], 2), round(current_pose[1], 2)+10]
        p1 = [round(current_pose[0], 2), round(current_pose[1], 2)]
        p2 = [round(goal[0],2),round(goal[1],2)]

        ''' 
        compute angle (in degrees) for p0p1p2 corner
        Inputs:
            p0,p1,p2 - points in the form of [x,y]
        '''

        v0 = np.array(p0) - np.array(p1)
        v1 = np.array(p2) - np.array(p1)


        angle = np.math.atan2(np.linalg.det([v1,v0]),np.dot(v1,v0))
        # print("angle ",np.degrees(angle), "radien : ",angle)
        # plt.plot(p0[0],p0[1],"ob")
        # plt.plot(p1[0],p1[1],"or")
        # plt.plot(p2[0],p2[1],"xr")
        # plt.plot(self.waypoint_x,self.waypoint_y,'b')
        # plt.show()

        return angle

    def do_scan(self, waypoint_x, waypoint_y, oz,distance_recording_cible,nbr_target_passing):
        self.waypoint_x=waypoint_x
        self.waypoint_y=waypoint_y
        desired_position = [0.0, 0.0, -oz[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        current_pose = self.current_pose
        desired_position[0], desired_position[1] = waypoint_x[self.mission_point], waypoint_y[self.mission_point]
        
       
        Bridge.deterministe_scan(self,current_pose)

        if abs(current_pose[0] - desired_position[0]) < 0.2 and abs(current_pose[1] - desired_position[1]) < 0.2 and self.flag_end==False:
            #print("Arrivé au point")
            
            self.mission_point += 1
            print( str(self.mission_point) + ' on ' + str(len(waypoint_x)-1))
            
            if self.mission_point > len(waypoint_x)-1 and self.flag_add_point==True:
                print("\n-------END SCAN----------")
               
                fichier = open("sonar_time.txt", "a")
                fichier.write(str(self.temps_4))
                fichier.write(",\n")
                fichier.write(str(self.temps_3))
                fichier.write(",\n")
                fichier.write(str(self.temps_2))
                fichier.write(",\n")
                fichier.write(str(self.temps_1))
                fichier.write(",\n")
                fichier.close()

                #take the time from the tirage.txt fille to connect it into the log and found the possition of the rov 
                #then found the target position, and create waypoint 
                x,y=return_on_target.target_coordinates(distance_recording_cible,nbr_target_passing)

                #add the new point into the waypoint_x and waypoint_y
                for j in range(0,len(x)):
                    for i in range(0,len(x[0])) :
                        waypoint_x.append(x[j][i])
                        waypoint_y.append(y[j][i])

                self.flag_add_point=False
            
            if self.flag_add_point==False :
                if  self.counter_start_stop == 0 :

                    self.counter_start_stop = 1

                else :
                    if self.counter_start_stop %2 != 0 :

                        print("start")
                        fichier = open("start_stop_recording.txt", "w")
                        fichier.write("start")
                        fichier.write(',\n')
                        fichier.close()
                        self.counter_start_stop = self.counter_start_stop + 1
                        
                    else :

                        print("stop")
                        fichier = open("start_stop_recording.txt", "w")
                        fichier.write("stop")
                        fichier.write(',\n')
                        fichier.close()
                        self.counter_start_stop = self.counter_start_stop + 1

            if self.mission_point == len(waypoint_x) and  self.flag_add_point==False:
                self.ok_pose = True
                
            else  :
                desired_position[9] = self.angle_to_point([waypoint_x[self.mission_point], waypoint_y[self.mission_point]])
                desired_position[0], desired_position[1] = waypoint_x[self.mission_point], waypoint_y[self.mission_point]
                self.mission_point_sent = False


        if self.mission_point_sent == False:
            time.sleep(0.05)
            # self.ok_pose = False
            self.set_position_target_local_ned(desired_position)
            time.sleep(0.05)
            self.mission_point_sent = True


        if self.ok_pose == True :
            
            if self.flag_end==False :
                print('----------END MISSION----------')
                self.mission_point = 0
                self.mission_point_sent = False 
                self.flag_end=True
                #hts_rov.dessin_historique()
           
        
    def do_calibrage(self,ox,oy,oz) :
       
        if self.flag_end==True  :
            return 
        
        current_pose = self.current_pose

        desired_position = [0.0, 0.0, -oz[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        desired_position[0], desired_position[1] = ox[self.mission_point], oy[self.mission_point]
        
        if abs(current_pose[0] - desired_position[0]) < 0.2 and abs(current_pose[1] - desired_position[1]) < 0.2 and self.flag_end==False:
            #print("Arrivé au point")
            
            self.mission_point += 1

            if self.mission_point < len(ox) :
                desired_position[0], desired_position[1] = ox[self.mission_point], oy[self.mission_point]
                self.mission_point_sent = False

            elif self.mission_point == len(ox) :
                self.mission_point = 0
                desired_position[0], desired_position[1] = ox[self.mission_point], oy[self.mission_point]
                self.mission_point_sent = False


        if self.mission_point_sent == False:
            print("ox = " + str(ox[self.mission_point])+ " oy = " + str( oy[self.mission_point]))
            time.sleep(0.05)
            # self.ok_pose = False
            self.set_position_target_local_ned(desired_position)
            time.sleep(0.05)
            self.mission_point_sent = True



    def loop_control(self):
	

        #print('ok')
        ok_z=False
        ok_yaw=False
        ok_xy=False
        #yaw_temp=100
        self.conn.set_mode('GUIDED')


        while(1):
            while not self.is_armed():
                self.conn.arducopter_arm()
        #leggo i miei dati
            x_rov=self.current_pose[0]
            y_rov=self.current_pose[1]
            z_rov=self.current_pose[2]
            roll_rov=self.current_pose[3]
            pitch_rov=self.current_pose[4]
            yaw_rov=math.degrees(self.current_pose[5])
            
      


if __name__ == '__main__':
    bridge = Bridge()
    
    while True:
        bridge.update()
        bridge.print_data()
        
        

#!/usr/bin/env python

from __future__ import division
import yaml
import numpy as np
import math
import rospy
import time
import navpy
from bridge import Bridge
from Gridy_based import Plannification
from geometry_msgs.msg import  PoseStamped
import time

try:
    from pubs import Pubs
    #from subs import Subs
#     from video import Video
except:
    from bluerov.pubs import Pubs
    #from bluerov.subs import Subs
#     from bluerov.video import Video

# from TrajectoryGenerator import TrajectoryGenerator

# convert opencv image to ros image msg
from cv_bridge import CvBridge

# msgs type

 
# from bluerov_ros_playground.msg import Bar30
# from bluerov_ros_playground.msg import Attitude 
# from bluerov_ros_playground.msg import State 

"""
Generates a quintic polynomial trajectory.
Author: Daniel Ingram (daniel-s-ingram)
"""


class TrajectoryGenerator():
    def __init__(self, start_pos, des_pos, T, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]
        
        self.T = T

    def solve(self):
        A = np.array(
            [[0, 0, 0, 0, 0, 1],
             [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
             [0, 0, 0, 0, 1, 0],
             [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
             [0, 0, 0, 2, 0, 0],
             [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
            ])

        b_x = np.array(
            [[self.start_x],
             [self.des_x],
             [self.start_x_vel],
             [self.des_x_vel],
             [self.start_x_acc],
             [self.des_x_acc]
            ])
        
        b_y = np.array(
            [[self.start_y],
             [self.des_y],
             [self.start_y_vel],
             [self.des_y_vel],
             [self.start_y_acc],
             [self.des_y_acc]
            ])

        b_z = np.array(
            [[self.start_z],
             [self.des_z],
             [self.start_z_vel],
             [self.des_z_vel],
             [self.start_z_acc],
             [self.des_z_acc]
            ])

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)


class BlueRov(Bridge):
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=115200):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.pub = Pubs()
        # self.sub = Subs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'
        self.flag_position_gps_depart=True

        # # self.video = Video()
        # self.video_bridge = CvBridge()

        self.pub_topics = [
            [
                self._create_position_msg,
                '/local_position',
                PoseStamped,
                1
            ]
          
        ]

      

        self.mavlink_msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.mavlink_msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

        
    @staticmethod

    def pub_pass(self):
        pass
   

    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
        """ Subscribe to a topic using the publisher

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
        """
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

   
    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    # TODO : tester l'utilisation des vélocités comme dans _create_odometry_msg

    def _create_position_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """
        
        # Check if data is available
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = PoseStamped()

        self._create_header(msg)

        # # http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        msg.pose.position.x = xyz_data[0]
        msg.pose.position.y = -xyz_data[1]
        msg.pose.position.z = - xyz_data[2]
        #print(xyz_data)
                                   
        # https://mavlink.io/en/messages/common.html#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)


        msg.pose.orientation.w = 1
        msg.pose.orientation.x = math.degrees(orientation[0])
        msg.pose.orientation.y = math.degrees(orientation[1])
        msg.pose.orientation.z = math.degrees(orientation[2])
        
        self.pub.set_data('/local_position', msg)





    


    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)

        # position = self.get_data()['LOCAL_POSITION_NED']
        # xyz_data = [position[i]  for i in ['x', 'y', 'z']]
        # x_courant = xyz_data[0]
        # y_courant = xyz_data[1]
        # z_courant = - xyz_data[2]



def get_yaml_data() :

    # !!! Write the path to your ymal file !!!
    
    Config_scan = "/home/eliott/Desktop/Bluerov2_dock_scan/Bluerov_scan_dock/bridge/Config_scan.yaml"
    with open(Config_scan,"r") as yaml_file: 
        data=yaml.full_load(yaml_file)

    #Scan settings 

    depth= 0
    range_sensor = 0
    scan_passing_accurary=0
    nbr_target_passing=0

    #GPS position 

    lat_ref=0
    lon_ref=0
    alt_ref=0
    point_gps_rov=[]
    point_gps=[]
    lat_long=[]


    # Get yaml data
    for name_item, data_item in data.items() :

        if name_item == 'depth' : 
            depth = data_item

        elif name_item == 'range_sensor' : 
            range_sensor = data_item

        elif name_item == 'scan_passing_accurary' : 
            scan_passing_accurary = data_item

        elif name_item == 'nbr_target_passing' : 
           nbr_target_passing = data_item

        elif name_item == 'lat_long' : 
            lat_long = data_item

        elif name_item == 'position_init_rov' : 
            point_gps_rov= data_item

    lat_ref=point_gps_rov[0]
    lon_ref=point_gps_rov[1]

    for i in range(len(lat_long)) : 
            point_gps.append(lat_long[i][0])
            point_gps.append(lat_long[i][1])


    print("\n----------SCAN SETTINGS----------")
    print("depth = ", depth)
    print("range_sensor = ", range_sensor )
    print("scan_passing_accurary = ", scan_passing_accurary)
    print("nbr_target_passing = ", nbr_target_passing)
    print("\n      POSITION INIT ROV")
    print("lat_ref = ", lat_ref)
    print("lon_ref = ", lon_ref)
    print("alt_ref = ", alt_ref)
    print("--------------------------------\n")



    return depth,range_sensor,scan_passing_accurary,nbr_target_passing,lon_ref,lat_ref,alt_ref,point_gps

def gps2ned() : 
    ox = []
    oy = []
    oz = [depth]


    # transform GPS into NED 
    for i in range(len(point_gps)-1):
        if i%2 == 0 : 

            x,y,z=navpy.lla2ned(point_gps[i],point_gps[i+1],0 ,lat_ref , lon_ref,alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84')
            
            ox.append(round(x))
            oy.append(round(y))
        
        
   
    ox.append(ox[0])
    oy.append(oy[0])  
    return ox,oy,oz

def log_initialisation():

    #ouvre les logs pour tout effacer
    print("initialisation log")
    fichier = open("log_position.txt", "w")
    fichier.write("-time---------------X------------------Y------------------Z------------------yaw------------------\n")
    fichier.close()
    #ouvre start_pose pour tout effacer 
    fichier = open("start_stop_recording.txt", "w")
    fichier.close()
    fichier = open("sonar_time.txt", "w")
    fichier.close()


if __name__ == '__main__':

    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    bluerov = BlueRov(device='udp:localhost:14551')
    

    # Get yaml data
    depth,range_sensor,scan_passing_accurary,nbr_target_passing,lon_ref,lat_ref,alt_ref,point_gps=get_yaml_data()

    # Transform gps point from yaml into ned point  
    ox,oy,oz=gps2ned()
   
    #algoythm of planning how create all the trajectory of the rov 
    plannification = Plannification()
    waypoint_x, waypoint_y = plannification.planning(ox, oy, scan_passing_accurary)
    waypoint_z=oz
    #clean and setup the log_positions.txt, start_pose.txt and tirage.txt files 
    log_initialisation()


    rate = rospy.Rate(50.0)
    print("\n---------ROV START SCAN---------")
   
    while not rospy.is_shutdown():

        bluerov.get_bluerov_position_data()

        bluerov.do_scan(waypoint_x, waypoint_y, waypoint_z, range_sensor,nbr_target_passing)
        #bluerov.do_calibrage( ox,oy,oz)

        bluerov.publish()

        rate.sleep()



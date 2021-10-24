#!/usr/bin/env python

'''
# Team name :       Swayamchalit Gaadi
# Filename:         PositionController6 
# Functions:        class Edrone(), gripper_active(state), is_at_setpoint3D(setpoint), is_at_setpoint2D(setpoint), stablize_drone(time_limit, position, speed)
                    stablize_drone(time_limit, position, speed), avoid_obstacle(position, speed), avoid_obstacle_new(position, speed), reach_short_destination(height_correction, position, speed, marker_detect)
                    reach_destination(height_correction, hc, position, speed, delivery), nearest(src), farthest(), make_box_position(), determine_order()
                    hover(), marker_detection(), generate_csv(), main()

# Global variables: none
'''

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, String, Int8, Float64
import rospy
import numpy as np
import rospkg
import csv
import time
import tf
import math

from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from vitarana_drone.srv import Gripper, GripperResponse
from sensor_msgs.msg import LaserScan


class Edrone():

    def __init__(self):
        '''
        Purpose:
        ---
        init function of Edrone class

        Input Arguments:
        ---
        self

        Returns:
        ---
        none

        Example call:
        ---
        none
        '''

        # initializing ros node with name position_controller
        rospy.init_node('position_controller6')

        # This will contain the current location of Edrone. [latitude, longitude, altitude ]
        # this value is updating each time in gps callback function
        self.drone_location = [0.0, 0.0, 0.0]

        # this value is updating each time in gps_velocity callback function
        self.current_velocity = [0.0, 0.0, 0.0]

        # To store the intermediate setpoints. [latitude , longitude, altitude ]
        self.setpoint_location = [19.0, 72.0, 3.0]

        # To store the intermediate setpoints. [latitude , longitude, altitude ]
        self.setpoint_velocity = [0.0, 0.0, 0.0]

        # To store the final setpoint. [latitude , longitude, altitude ]
        self.setpoint_final = [19.0, 72.0, 3.0]

        # To store the initial position. [latitude , longitude, altitude ]
        self.setpoint_initial = [19.0, 72.0, 3.0]

        # This is the location of destination of box retrive from qr code. [latitude , longitude, altitude ]
        self.box_position = [0.0, 0.0, 0.0]

        # This will contain the current orientation of eDrone in quaternion format. [x,y,z,w]
        # This value is updating each time in imu callback function
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This will contain the current orientation of eDrone converted in euler angles form. [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # To store values of LIDAR
        self.laser_negative_latitude = 0
        self.laser_positive_longitude = 0
        self.laser_negative_longitude = 0
        self.laser_positive_latitude = 0

        # Declaring rpyt_cmd of message type edrone_cmd and initializing values
        # { rpyt_cmd --> roll, pitch, yaw, throttle command}
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        # Declaring error values and tolerences to publish for visualization in plotjuggler
        self.latitude_Error = Float32()
        self.latitude_Error.data = 0.0
        self.latitude_Up = Float32()
        self.latitude_Up.data = 0.0000045173
        self.latitude_Low = Float32()
        self.latitude_Low.data = -0.0000045173

        self.longitude_Error = Float32()
        self.longitude_Error.data = 0.0
        self.longitude_Up = Float32()
        self.longitude_Up.data = 0.0000047483
        self.longitude_Low = Float32()
        self.longitude_Low.data = -0.0000047483

        self.altitude_Error = Float32()
        self.altitude_Error.data = 0.0
        self.altitude_Up = Float32()
        self.altitude_Up.data = 0.2
        self.altitude_Low = Float32()
        self.altitude_Low.data = -0.2

        # Declaring variable to store different error values, to be used in PID equations.
        self.change_in_error_value = [0.0, 0.0, 0.0]
        self.error_value = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.sum_error_value = [0.0, 0.0, 0.0]

        # Declaring maximum and minimum values for roll, pitch, yaw, throttle output.
        self.max_values = [2000.0, 2000.0, 2000.0, 2000.0]
        self.min_values = [1000.0, 1000.0, 1000.0, 1000.0]

        # initializing Kp, Kd and ki for position [latitude, longitude, altitude] after tunning
        self.Kp_p = [1080000, 1140000, 48]
        self.Ki_p = [0, 0, 0]
        self.Kd_p = [57600000, 57900000, 3000]

        # initializing Kp, Kd and ki for velocity [latitude, longitude, altitude] after tunning
        self.Kp_v = [2.156, 2.156, 900]  # 650]
        self.Ki_v = [0, 0, 0]
        self.Kd_v = [151, 151, 500]  # 25]

        # Declaring variables for marker midpoint
        self.centre_x = -1
        self.centre_y = -1

        # Variable for storing Range Finder Bottom data
        self.vertical_distance = 0

        # Storing drone original coordinates
        self.initial_location = [18.9998102845, 72.000142461, 16.757981]
        self.box = {
            'A1': [18.9998102845, 72.000142461, 16.757981]
        }
        self.patch = {
            'X1':  [18.9999367615, 72.000142461, 16.757981]
        }

        rospack = rospkg.RosPack()
        filepath = rospack.get_path('vitarana_drone')+'/scripts/bonus.csv'

        # Reading mainfest.csv and storing delivery and return seperately
        self.delivery_location = {}
        self.return_location = {}
        self.visited = {}
        self.order = []
        self.csvrow = []
        with open(filepath) as manifest:
            csvread = csv.reader(manifest)
            for row in csvread:
                if(row[0] == "DELIVERY"):
                    self.delivery_location[row[1]] = row[2].split(";")
                else:
                    try:
                        i, _ = row[-1].split(" ")
                    except:
                        i = row[-1]
                    self.return_location[i] = row[1].split(";")

            for i in self.delivery_location.keys():
                self.delivery_location[i][0] = float(
                    self.delivery_location[i][0])
                self.delivery_location[i][1] = float(
                    self.delivery_location[i][1])
                self.delivery_location[i][2] = float(
                    self.delivery_location[i][2])
                self.visited[i] = False

            for i in self.return_location.keys():
                self.return_location[i][0] = float(self.return_location[i][0])
                self.return_location[i][1] = float(self.return_location[i][1])
                self.return_location[i][2] = float(self.return_location[i][2])
                self.visited[i] = False

        self.current_marker_id = ""

        # initializing Publisher for /drone_command, /latitude_error, /longitude_error, /altitude_error and tolerences
        self.rpyt_pub = rospy.Publisher(
            '/drone_command', edrone_cmd, queue_size=1)
        self.latitude_error = rospy.Publisher(
            '/latitude_error', Float32, queue_size=1)
        self.longitude_error = rospy.Publisher(
            '/longitude_error', Float32, queue_size=1)
        self.altitude_error = rospy.Publisher(
            '/altitude_error', Float32, queue_size=1)

        self.latitude_up = rospy.Publisher(
            '/latitude_up', Float32, queue_size=1)
        self.longitude_up = rospy.Publisher(
            '/longitude_up', Float32, queue_size=1)
        self.altitude_up = rospy.Publisher(
            '/altitude_up', Float32, queue_size=1)
        self.latitude_low = rospy.Publisher(
            '/latitude_low', Float32, queue_size=1)
        self.longitude_low = rospy.Publisher(
            '/longitude_low', Float32, queue_size=1)
        self.altitude_low = rospy.Publisher(
            '/altitude_low', Float32, queue_size=1)

        self.curr_marker_id = rospy.Publisher(
            '/edrone/curr_marker_id', String, queue_size=1)
        self.yaw_pub = rospy.Publisher('/edrone/yaw', Float64, queue_size=1)

        # Subscribing to /edrone/gps, /pid_tuning_yawyawroll, /pid_tuning_pitch, /pid_tuning_yaw {used these GUIs only to tune ;-) }
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/edrone/gps_velocity',
                         Vector3Stamped, self.velocity_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan,
                         self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',
                         LaserScan, self.range_finder_bottom_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)        # for latitude
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)      # for longitude
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)          # for altitude
        rospy.Subscriber('/edrone/marker_data', MarkerData,
                         self.marker_data_callback)

    def range_finder_top_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for range finder sensor

        Input Arguments:
        ---
        self
        msg : [list]
            contains the ranges by the sensor

        Returns:
        ---
        none

        Example call:
        ---
        self.range_finder_top_callback
        '''

        if(self.drone_orientation_euler[2] < np.pi/4 and self.drone_orientation_euler[2] > -np.pi/4):
            self.laser_negative_longitude, self.laser_positive_latitude, self.laser_positive_longitude, self.laser_negative_latitude, _ = msg.ranges

        elif(self.drone_orientation_euler[2] < 3*np.pi/4 and self.drone_orientation_euler[2] > np.pi/4):
            self.laser_negative_latitude, self.laser_negative_longitude, self.laser_positive_latitude, self.laser_positive_longitude, _ = msg.ranges

        elif(self.drone_orientation_euler[2] > 3*np.pi/4 or self.drone_orientation_euler[2] < -3*np.pi/4):
            self.laser_positive_longitude, self.laser_negative_latitude, self.laser_negative_longitude, self.laser_positive_latitude, _ = msg.ranges

        elif(self.drone_orientation_euler[2] > -3*np.pi/4 and self.drone_orientation_euler[2] < -np.pi/4):
            self.laser_positive_latitude, self.laser_positive_longitude, self.laser_negative_latitude, self.laser_negative_longitude, _ = msg.ranges

    def range_finder_bottom_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for range finder bottom sensor

        Input Arguments:
        ---
        self
        msg : [list]
            contains the ranges by the sensor

        Returns:
        ---
        none

        Example call:
        ---
        self.range_finder_bottom_callback
        '''
        if(msg.ranges[0] < 50 and msg.ranges[0] >= 0.5):
            self.vertical_distance = msg.ranges[0]
        if(self.current_marker_id == 'C1'):
            self.vertical_distance = self.vertical_distance - 1.6

    def marker_data_callback(self, marker_data):
        '''
        Purpose:
        ---
        Call back fuction for marker_data

        Input Arguments:
        ---
        self
        marker_data : [dictionary]
            contains the marker data published by detectMarker node

        Returns:
        ---
        none

        Example call:
        ---
        self.marker_data_callback
        '''

        self.centre_x = self.drone_location[0] + \
            marker_data.err_x_m*0.0000045173*2

        self.centre_y = self.drone_location[1] - \
            marker_data.err_y_m*0.0000047483*2

    def imu_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for IMU

        Input Arguments:
        ---
        self
        msg : [dictionary]
            contains the data by IMU sensor

        Returns:
        ---
        none

        Example call:
        ---
        self.imu_callback
        '''

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        # converting the current orientations from quaternion to euler angles
        # 2 is yaw
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

    def velocity_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for velocity

        Input Arguments:
        ---
        self
        msg : [dictionary]
            contains the velocity by GPS

        Returns:
        ---
        none

        Example call:
        ---
        self.vleocity_callback
        '''

        self.current_velocity[0] = msg.vector.x
        self.current_velocity[1] = -msg.vector.y
        self.current_velocity[2] = msg.vector.z

    def gps_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for gps data

        Input Arguments:
        ---
        self
        msg : [dictionary]
            contains the gps coordinate publicshed by gps module

        Returns:
        ---
        none

        Example call:
        ---
        self.gps_callback
        '''

        self.drone_location[0] = msg.latitude
        self.drone_location[1] = msg.longitude
        self.drone_location[2] = msg.altitude

    def roll_set_pid(self, roll):
        '''
        Purpose:
        ---
        Call back fuction for roll_set_pid

        Input Arguments:
        ---
        self
        roll : [dictionary]
            contains the marker data published by pid sliders

        Returns:
        ---
        none

        Example call:
        ---
        self.roll_set_pid
        '''

        self.Kp_v[0] = roll.Kp * 0.001
        self.Ki_v[0] = roll.Ki * 0.0001
        self.Kd_v[0] = roll.Kd * 1

    def pitch_set_pid(self, pitch):
        '''
        Purpose:
        ---
        Call back fuction for Pitch_set_pid

        Input Arguments:
        ---
        self
        pitch : [dictionary]
            contains the marker data published by pid silder

        Returns:
        ---
        none

        Example call:
        ---
        self.pitch_set_pid
        '''

        self.Kp_v[1] = pitch.Kp * 0.001
        self.Ki_v[1] = pitch.Ki * 0.0001
        self.Kd_v[1] = pitch.Kd * 1

    def yaw_set_pid(self, yaw):
        '''
        Purpose:
        ---
        Call back fuction for yaw_set_pid

        Input Arguments:
        ---
        self
        yaw : [dictionary]
            contains the marker data published by pid sliders

        Returns:
        ---
        none

        Example call:
        ---
        self.yaw_set_pid
        '''

        self.Kp_v[2] = yaw.Kp * 1
        self.Ki_v[2] = yaw.Ki * 1
        self.Kd_v[2] = yaw.Kd * 1

    def publish_data(self):
        '''
        Purpose:
        ---
        to publish all the publisher of the this node

        Input Arguments:
        ---
        self

        Returns:
        ---
        none

        Example call:
        ---
        self.publish_data() , e_drone.publish_data()
        '''
        # publishing rpyt_cmd to /drone_command
        self.rpyt_pub.publish(self.rpyt_cmd)

        # publishing different error values and tolerences
        self.latitude_error.publish(self.latitude_Error)
        self.longitude_error.publish(self.longitude_Error)
        self.altitude_error.publish(self.altitude_Error)
        self.latitude_up.publish(self.latitude_Up)
        self.longitude_up.publish(self.longitude_Up)
        self.altitude_up.publish(self.altitude_Up)
        self.latitude_low.publish(self.latitude_Low)
        self.longitude_low.publish(self.longitude_Low)
        self.altitude_low.publish(self.altitude_Low)
        self.curr_marker_id.publish(self.current_marker_id)
        self.yaw_pub.publish(self.drone_orientation_euler[2])

    def pid(self, position, speed):
        '''
        Purpose:
        ---
        The PID loop

        Input Arguments:
        ---
        self
        position : [bool]
            true ---> position controller,  false --> velocity controller
        speed : [float]
            maximum allowed speed

        Returns:
        ---
        none

        Example call:
        ---
        e_drone.pid(position=False, speed=100)
        '''

        # if position is true, the drone will be position controlled else velocity controlled

        if(position):
            for i in range(3):
                self.error_value[i] = self.setpoint_location[i] - \
                    self.drone_location[i]
                self.sum_error_value[i] = self.sum_error_value[i] + \
                    self.error_value[i]
                self.change_in_error_value[i] = self.error_value[i] - \
                    self.prev_error_value[i]
                self.prev_error_value[i] = self.error_value[i]

            # assigning error value to its container to publish
            self.latitude_Error.data = self.error_value[0]
            self.longitude_Error.data = self.error_value[1]
            self.altitude_Error.data = self.error_value[2]

            # PID eqation for latitude
            output0 = self.Kp_p[0]*self.error_value[0] + self.Ki_p[0] * \
                self.sum_error_value[0] + self.Kd_p[0] * \
                self.change_in_error_value[0]

            # PID eqation for longitude
            output1 = self.Kp_p[1]*self.error_value[1] + self.Ki_p[1] * \
                self.sum_error_value[1] + self.Kd_p[1] * \
                self.change_in_error_value[1]

            # PID equation for altitude
            output2 = self.Kp_p[2]*self.error_value[2] + self.Ki_p[2] * \
                self.sum_error_value[2] + self.Kd_p[2] * \
                self.change_in_error_value[2]

        else:
            self.setpoint_velocity[0] = (
                self.setpoint_location[0] - self.drone_location[0])/0.0000045173*2
            self.setpoint_velocity[1] = (
                self.setpoint_location[1] - self.drone_location[1])/0.0000047483*2
            self.setpoint_velocity[2] = (
                self.setpoint_location[2] - self.drone_location[2])

            x = np.sqrt(
                self.setpoint_velocity[0]**2 + self.setpoint_velocity[1]**2 + self.setpoint_velocity[2]**2)

            # decreasing velocity if greater than given speed
            if(x > speed):
                self.setpoint_velocity[0] = self.setpoint_velocity[0]*speed/x
                self.setpoint_velocity[1] = self.setpoint_velocity[1]*speed/x
                self.setpoint_velocity[2] = self.setpoint_velocity[2]*speed/x

            for i in range(3):
                if(i == 2):
                    self.error_value[i] = self.setpoint_velocity[i] - \
                        self.current_velocity[i]
                else:
                    self.error_value[i] = self.setpoint_velocity[i] - \
                        1.4*self.current_velocity[i]
                self.sum_error_value[i] = self.sum_error_value[i] + \
                    self.error_value[i]
                self.change_in_error_value[i] = self.error_value[i] - \
                    self.prev_error_value[i]
                self.prev_error_value[i] = self.error_value[i]

            # assigning error value to its container to publish
            self.latitude_Error.data = self.setpoint_location[0] - \
                self.drone_location[0]
            self.longitude_Error.data = self.setpoint_location[1] - \
                self.drone_location[1]
            self.altitude_Error.data = self.setpoint_location[2] - \
                self.drone_location[2]

            # PID eqation for latitude
            output0 = self.Kp_v[0]*self.error_value[0] + self.Ki_v[0] * \
                self.sum_error_value[0] + self.Kd_v[0] * \
                self.change_in_error_value[0]

            # PID eqation for longitude
            output1 = self.Kp_v[1]*self.error_value[1] + self.Ki_v[1] * \
                self.sum_error_value[1] + self.Kd_v[1] * \
                self.change_in_error_value[1]

            # PID equation for altitude
            output2 = self.Kp_v[2]*self.error_value[2] + self.Ki_v[2] * \
                self.sum_error_value[2] + self.Kd_v[2] * \
                self.change_in_error_value[2]

        # updating the roll value according to PID output.
        # {this equation will work fine when there is some yaw. to see detail opne --> https://drive.google.com/file/d/14gjse4HUIi9OoznefjOehh1HU6LUhoO7/view?usp=sharing }
        self.rpyt_cmd.rcRoll = 1500 + output0

        # updating the pitch value according to PID output
        self.rpyt_cmd.rcPitch = 1500 + output1

        # updating the throttle value according to PID output
        self.rpyt_cmd.rcThrottle = 1500 + output2

        # checking the boundary conditions for roll value
        if(self.rpyt_cmd.rcRoll > 1800):
            self.rpyt_cmd.rcRoll = 1800
        elif(self.rpyt_cmd.rcRoll < 1200):
            self.rpyt_cmd.rcRoll = 1200

        # checking the boundary conditions for pitch value
        if(self.rpyt_cmd.rcPitch > 1800):
            self.rpyt_cmd.rcPitch = 1800
        elif(self.rpyt_cmd.rcPitch < 1200):
            self.rpyt_cmd.rcPitch = 1200

        # checking the boundary conditions for throttle value
        if(self.rpyt_cmd.rcThrottle > 2000):
            self.rpyt_cmd.rcThrottle = 2000
        elif(self.rpyt_cmd.rcThrottle < 1000):
            self.rpyt_cmd.rcThrottle = 1000

        # self.rpyt_cmd.rcYaw = 2000.0
        self.publish_data()


def gripper_active(state):
    '''
    Purpose:
    ---
    activate and deactivate the griper

    Input Arguments:
    ---
    state : [bool]
        ture --> the griper will be activated vice versa

    Returns:
    ---
    resp : [bool]
        responce by the groper node

    Example call:
    ---
    gripper_active(1)
    '''

    rospy.wait_for_service('/edrone/activate_gripper')
    try:
        gripper_state = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        if(state == 1):
            resp = gripper_state(activate_gripper=True)
        else:
            resp = gripper_state(activate_gripper=False)
            # return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def is_at_setpoint3D(setpoint):
    '''
    Purpose:
    ---
    will tell if the drone is under the threshold at set point in all three directions

    Input Arguments:
    ---
    setpoint : [list]
        setpoint (destination)

    Returns:
    ---
    return : [bool]
        true --> withine threeshold or vice versa

    Example call:
    ---
    is_at_setpoint3D(e_drone.setpoint)    
    '''
    return ((e_drone.drone_location[0] > setpoint[0]+0.0000045173/4 or
             e_drone.drone_location[0] < setpoint[0]-0.0000045173/4) or
            (e_drone.drone_location[1] > setpoint[1]+0.0000047483/4 or
             e_drone.drone_location[1] < setpoint[1]-0.0000047483/4) or
            (e_drone.drone_location[2] > setpoint[2]+0.2 or
             e_drone.drone_location[2] < setpoint[2]-0.2))


def is_at_setpoint2D(setpoint):
    '''
    Purpose:
    ---
    will tell if the drone is under the threshold at set point in x, y direction (horizontal plane)

    Input Arguments:
    ---
    setpoint : [list]
        setpoint (destination)

    Returns:
    ---
    return : [bool]
        true --> withine threeshold or vice versa

    Example call:
    ---
    is_at_setpoint2D(e_drone.setpoint)    
    '''

    return ((e_drone.drone_location[0] > setpoint[0]+0.0000045173/5 or
             e_drone.drone_location[0] < setpoint[0]-0.0000045173/5) or
            (e_drone.drone_location[1] > setpoint[1]+0.0000047483/5 or
             e_drone.drone_location[1] < setpoint[1]-0.0000047483/5))


def stablize_drone(time_limit, position, speed):
    '''
    Purpose:
    ---
    to stablize the drone around a given point

    Input Arguments:
    ---
    time_limit : [float]
        time upto whitch stablization performed

    position : [bool]
        true ---> position controller,  false --> velocity controller

    speed : [float]
        maximum allowed speed

    Returns:
    ---
    none

    Example call:
    ---
    stablize_drone(time_limit = 10, position = False, speed = 100) 
    '''
    t = time.time()
    while time.time() - t < time_limit:
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.05)


def avoid_obstacle(position, speed):
    '''
    Purpose:
    ---
    to avoid the obsltracle (using modified bug algorithm)

    Input Arguments:
    ---
    position : [bool]
        true ---> position controller,  false --> velocity controller

    speed : [float]
        maximum allowed speed

    Returns:
    ---
    none

    Example call:
    ---
    avoid_obstacle(position = False, speed = 100) 
    '''

    flag = 0
    m = 15
    x = 35
    # Obstacle on negative latitude and destination in negative latitude
    while(e_drone.laser_negative_latitude <= 10 and e_drone.laser_negative_latitude >= 0.5 and
            e_drone.setpoint_final[0] - e_drone.drone_location[0] <= 0):
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.05)
        e_drone.setpoint_location[1] = e_drone.drone_location[1] + 0.0000020*m * \
            np.sign(e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])
        flag = 1

        if((e_drone.laser_negative_longitude <= 6 and e_drone.laser_negative_longitude >= 0.5) or
                (e_drone.laser_positive_longitude <= 6 and e_drone.laser_positive_longitude >= 0.5)):
            flag = 5
            break

    # Move the drone further away from obstacle ( Corner Case )
    if(flag == 1):
        for _ in range(x):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)
            e_drone.setpoint_location[1] = e_drone.drone_location[1] + 0.0000025*np.sign(
                e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])

    # Obstacle on positive latitude and destination in positive latitude
    while(e_drone.laser_positive_latitude <= 10 and e_drone.laser_positive_latitude >= 0.5 and
          e_drone.setpoint_final[0] - e_drone.drone_location[0] >= 0):
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.05)
        e_drone.setpoint_location[1] = e_drone.drone_location[1] - 0.0000020*m * \
            np.sign(e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])
        flag = 2

        if((e_drone.laser_negative_longitude <= 6 and e_drone.laser_negative_longitude >= 0.5) or
                (e_drone.laser_positive_longitude <= 6 and e_drone.laser_positive_longitude >= 0.5)):
            flag = 5
            break

    # Move the drone further away from obstacle ( Corner Case )
    if(flag == 2):
        for _ in range(x):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)
            e_drone.setpoint_location[1] = e_drone.drone_location[1] - 0.0000025*np.sign(
                e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])

    # Obstacle on negative longitude and destination in negative longitude
    while(e_drone.laser_negative_longitude <= 10 and e_drone.laser_negative_longitude >= 0.5 and
            e_drone.setpoint_final[1] - e_drone.drone_location[1] <= 0):
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.05)
        # print(e_drone.laser_negative_longitude)
        e_drone.setpoint_location[0] = e_drone.drone_location[0] + 0.0000020*m * \
            np.sign(e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])
        flag = 3

        if((e_drone.laser_negative_latitude <= 6 and e_drone.laser_negative_latitude >= 0.5) or
                (e_drone.laser_positive_latitude <= 6 and e_drone.laser_positive_latitude >= 0.5)):
            flag = 5
            break

    # Move the drone further away from obstacle ( Corner Case )
    if(flag == 3):
        for _ in range(x):
            # print(e_drone.laser_negative_longitude)
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)
            e_drone.setpoint_location[0] = e_drone.drone_location[0] + 0.0000020*m*np.sign(
                e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])

    # Obstacle on positive longitude and destination is positive longitude
    while(e_drone.laser_positive_longitude <= 10 and e_drone.laser_positive_longitude >= 0.5 and
            e_drone.setpoint_final[1] - e_drone.drone_location[1] >= 0):
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.05)
        e_drone.setpoint_location[0] = e_drone.drone_location[0] + 0.0000020*m * \
            np.sign(e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])
        flag = 4

        if((e_drone.laser_negative_latitude <= 6 and e_drone.laser_negative_latitude >= 0.5) or
                (e_drone.laser_positive_latitude <= 6 and e_drone.laser_positive_latitude >= 0.5)):
            flag = 5
            break

    if(flag == 4):
        for _ in range(x):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)
            e_drone.setpoint_location[0] = e_drone.drone_location[0] + 0.0000020*m*np.sign(
                e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])

    # If Obstacle on 2 sides, increase altitude till there is no obstacle
    if(flag == 5):
        while((e_drone.laser_negative_latitude <= 6 and e_drone.laser_negative_latitude >= 0.5) or
                (e_drone.laser_positive_latitude <= 6 and e_drone.laser_positive_latitude >= 0.5) or
              (e_drone.laser_negative_longitude <= 4 and e_drone.laser_negative_longitude >= 0.5) or
              (e_drone.laser_negative_longitude <= 4 and e_drone.laser_negative_longitude >= 0.5)):

            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)
            e_drone.setpoint_location = e_drone.setpoint_location[:-1] + [
                e_drone.drone_location[2] + 1]

        e_drone.setpoint_location = e_drone.setpoint_location[:-1] + [
            e_drone.drone_location[2] + 3]
        while (is_at_setpoint3D(e_drone.setpoint_location)):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)


def avoid_obstacle_new(position, speed):
    '''
    Purpose:
    ---
    to avoid the obstacle (raising the drone upwards if obstacle detected)

    Input Arguments:
    ---
    position : [bool]
        true ---> position controller,  false --> velocity controller

    speed : [float]
        maximum allowed speed

    Returns:
    ---
    none

    Example call:
    ---
    avoid_obstacle_new(position = False, speed = 100) 
    '''

    flag2 = 0
    while((e_drone.laser_negative_latitude <= 4.3 and e_drone.laser_negative_latitude >= 0.5 or
           e_drone.laser_negative_longitude <= 4.3 and e_drone.laser_negative_longitude >= 0.5 or
           e_drone.laser_positive_latitude <= 4.3 and e_drone.laser_positive_latitude >= 0.5 or
           e_drone.laser_positive_longitude <= 4.3 and e_drone.laser_positive_longitude >= 0.5)):
        multiplier = 0.00022
        if(np.sqrt(e_drone.current_velocity[0]**2 + e_drone.current_velocity[1]**2 + e_drone.current_velocity[2]**2) >= 1):
            while(np.sqrt(e_drone.current_velocity[0]**2 + e_drone.current_velocity[1]**2 + e_drone.current_velocity[2]**2) >= 0.5):
                e_drone.pid(position=position, speed=speed)
                time.sleep(0.05)
                e_drone.setpoint_location[0] = e_drone.drone_location[0] - 0.0000090*1000 * e_drone.current_velocity[0]/np.sqrt(
                    e_drone.current_velocity[0]**2 + e_drone.current_velocity[1]**2)
                e_drone.setpoint_location[1] = e_drone.drone_location[1] - 0.0000090*1000 * e_drone.current_velocity[1]/np.sqrt(
                    e_drone.current_velocity[0]**2 + e_drone.current_velocity[1]**2)

        e_drone.setpoint_location[0] = e_drone.drone_location[0]
        e_drone.setpoint_location[1] = e_drone.drone_location[1]
        e_drone.setpoint_location[2] = e_drone.setpoint_location[2] + 0.5
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.03)
        flag2 = 1

    if(flag2):
        e_drone.setpoint_location = e_drone.drone_location[:-1] + [
            e_drone.drone_location[2]+1]
        t = time.time()
        while time.time() - t < 0.5:
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)
        while is_at_setpoint2D(e_drone.setpoint_final):
            e_drone.pid(position=False, speed=100)
            time.sleep(0.05)
            divider = np.sqrt((e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])**2 + (
                e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])**2)
            if((e_drone.setpoint_location[0] > e_drone.setpoint_final[0]+0.0000340517 or
                e_drone.setpoint_location[0] < e_drone.setpoint_final[0]-0.0000340517) or
                    (e_drone.setpoint_location[1] > e_drone.setpoint_final[1]+0.00003407487 or
                        e_drone.setpoint_location[1] < e_drone.setpoint_final[1]-0.00003407487)):
                e_drone.setpoint_location[0] = e_drone.drone_location[0] + multiplier*(
                    e_drone.setpoint_final[0] - e_drone.drone_location[0]) / divider
                e_drone.setpoint_location[1] = e_drone.drone_location[1] + multiplier*(
                    e_drone.setpoint_final[1] - e_drone.drone_location[1]) / divider
            else:
                e_drone.setpoint_location[0] = e_drone.setpoint_final[0]
                e_drone.setpoint_location[1] = e_drone.setpoint_final[1]


def reach_short_destination(height_correction, position, speed, marker_detect):
    '''
    Purpose:
    ---
    to reaching shoet destinations

    Input Arguments:
    ---
    height_correction : [flot]
        amount of change in altitude(in meters) during the flights

    position : [bool]
        true ---> position controller,  false --> velocity controller

    speed : [flot]
        maximum allowed speed

    marker_detect : [bool]
        true --> detect the marker

    Returns:
    ---
    none

    Example call:
    ---
    reach_short_destination(height_correction = 5.0 , position = False, speed = 100, marker_detect = False) 
    '''

    if(height_correction):
        if(e_drone.setpoint_final[2] + height_correction > e_drone.drone_location[2]):
            e_drone.setpoint_location = e_drone.setpoint_final[:-1] + [
                e_drone.setpoint_final[2] + height_correction]
        else:
            e_drone.setpoint_location = e_drone.setpoint_final[:-1] + [
                e_drone.drone_location[2] + 3]

        while is_at_setpoint2D(e_drone.setpoint_location):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)

    elif(marker_detect):
        e_drone.setpoint_location = e_drone.setpoint_final[:-1] + [
            e_drone.drone_location[2]]
        while is_at_setpoint2D(e_drone.setpoint_location):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.04)

    stablize_drone(time_limit=2, position=position, speed=speed)

    e_drone.setpoint_location = e_drone.setpoint_final
    while is_at_setpoint3D(e_drone.setpoint_location):
        e_drone.pid(position=position, speed=speed)
        time.sleep(0.03)


def reach_destination(height_correction, hc, position, speed, delivery):
    '''
    Purpose:
    ---
    to reaching a given destination in a st. line 

    Input Arguments:
    ---
    height_correction : [float]
        amount of change in altitude(in meters) before the flights

    hc : [bool]
        land or decent to given height at the end of flight

    position : [bool]
        true ---> position controller,  false --> velocity controller

    speed : [float]
        maximum allowed speed

    delivery : [bool]
        true --> whether to deliver or not in the flight

    Returns:
    ---
    none

    Example call:
    ---
    reach_destination(height_correction = 5.0, hc = 0, position = False, speed = 100, delivery = True) 
    '''

    if(not position):
        e_drone.setpoint_location = e_drone.drone_location[:-1] + [
            e_drone.drone_location[-1]+6]
        t = time.time()

        while time.time() - t < 0.9:
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)

        e_drone.setpoint_initial[0] = e_drone.drone_location[0]
        e_drone.setpoint_initial[1] = e_drone.drone_location[1]
        e_drone.setpoint_initial[2] = e_drone.drone_location[2]
        e_drone.setpoint_location[0] = e_drone.setpoint_initial[0]
        e_drone.setpoint_location[1] = e_drone.setpoint_initial[1]
        e_drone.setpoint_location[2] = e_drone.setpoint_initial[2] + \
            height_correction
        multiplier = 0.00022
        # multiplier = 0.000011*20

        while(is_at_setpoint2D(e_drone.setpoint_final)):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)

            divider = np.sqrt((e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])**2 + (
                e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])**2)

            if((e_drone.setpoint_location[0] > e_drone.setpoint_final[0]+0.0000340517 or
                e_drone.setpoint_location[0] < e_drone.setpoint_final[0]-0.0000340517) or
                    (e_drone.setpoint_location[1] > e_drone.setpoint_final[1]+0.00003407487 or
                        e_drone.setpoint_location[1] < e_drone.setpoint_final[1]-0.00003407487)):

                e_drone.setpoint_location[0] = e_drone.drone_location[0] + multiplier*(
                    e_drone.setpoint_final[0] - e_drone.drone_location[0]) / divider
                e_drone.setpoint_location[1] = e_drone.drone_location[1] + multiplier*(
                    e_drone.setpoint_final[1] - e_drone.drone_location[1]) / divider

            else:
                e_drone.setpoint_location[0] = e_drone.setpoint_final[0]
                e_drone.setpoint_location[1] = e_drone.setpoint_final[1]

            if(delivery or True):
                e_drone.setpoint_location[2] = max(
                    (e_drone.setpoint_location[2] - 0.03), e_drone.setpoint_final[2] + 9)

            avoid_obstacle_new(position=position, speed=speed)

        if not delivery:
            stablize_drone(time_limit=2, position=position, speed=speed)

        if(hc):
            e_drone.setpoint_location = e_drone.setpoint_final[:-1] + [
                e_drone.setpoint_final[2]]
            while is_at_setpoint3D(e_drone.setpoint_location):
                e_drone.pid(position=position, speed=speed)
                time.sleep(0.04)

    else:
        if(height_correction):
            # Incease altitude of drone to 30 if it is lower
            if(e_drone.setpoint_final[2] > e_drone.drone_location[2] or e_drone.drone_location[2] < 24):
                e_drone.setpoint_location = e_drone.drone_location[:-1] + [30]
                while(is_at_setpoint3D(e_drone.setpoint_location)):
                    e_drone.pid(position=position, speed=speed)
                    time.sleep(0.05)
            stablize_drone(time_limit=5, position=position, speed=speed)

        e_drone.setpoint_initial[0] = e_drone.drone_location[0]
        e_drone.setpoint_initial[1] = e_drone.drone_location[1]
        e_drone.setpoint_initial[2] = e_drone.drone_location[2]
        e_drone.setpoint_location[0] = e_drone.setpoint_initial[0]
        e_drone.setpoint_location[1] = e_drone.setpoint_initial[1]
        e_drone.setpoint_location[2] = e_drone.setpoint_initial[2]
        multiplier = 0.000010*speed

        while(is_at_setpoint2D(e_drone.setpoint_final)):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)

            divider = np.sqrt((e_drone.setpoint_final[0] - e_drone.setpoint_initial[0])**2 + (
                e_drone.setpoint_final[1] - e_drone.setpoint_initial[1])**2)

            if((e_drone.setpoint_location[0] > e_drone.setpoint_final[0]+0.0000150517 or
                e_drone.setpoint_location[0] < e_drone.setpoint_final[0]-0.0000150517) or
                    (e_drone.setpoint_location[1] > e_drone.setpoint_final[1]+0.00001507487 or
                        e_drone.setpoint_location[1] < e_drone.setpoint_final[1]-0.00001507487)):

                e_drone.setpoint_location[0] = e_drone.drone_location[0] + multiplier*(
                    e_drone.setpoint_final[0] - e_drone.drone_location[0]) / divider
                e_drone.setpoint_location[1] = e_drone.drone_location[1] + multiplier*(
                    e_drone.setpoint_final[1] - e_drone.drone_location[1]) / divider

            else:
                e_drone.setpoint_location[0] = e_drone.setpoint_final[0]
                e_drone.setpoint_location[1] = e_drone.setpoint_final[1]

        stablize_drone(time_limit=2, position=position, speed=speed)

        # Descending the drone on the setpoint
        e_drone.setpoint_location = e_drone.setpoint_final
        while (is_at_setpoint3D(e_drone.setpoint_final)):
            e_drone.pid(position=position, speed=speed)
            time.sleep(0.05)

# Finds the nearest delivery/return


def nearest(src):
    '''
    Purpose:
    ---
    to find the nearest package from the drone

    Input Arguments:
    ---
    src : [list]

    Returns:
    ---
    absolute value of distance of nearest delivery location

    Example call:
    ---
    nearest(src)
    '''

    ans = ''
    a = 1000000000000
    for i in e_drone.return_location.keys():
        if(not e_drone.visited[i]):
            curr = np.sqrt(((src[0]-e_drone.return_location[i][0])*1.5/0.000013552)
                           ** 2 + ((src[1]-e_drone.return_location[i][1])*1.5/0.000014245)**2)
            if(curr < a):
                ans = i
                a = curr

    e_drone.visited[ans] = True
    return ans


def farthest():
    '''
    Purpose:
    ---
    to find the farthest package from the drone

    Input Arguments:
    ---
    none

    Returns:
    ---
    absolute value of distance of farthest delivery location

    Example call:
    ---
    farthest()
    '''

    ans = ''
    a = 0
    for i in e_drone.delivery_location.keys():
        if(not e_drone.visited[i]):
            curr = np.sqrt(((e_drone.initial_location[0]-e_drone.delivery_location[i][0])*1.5/0.000013552)**2 + (
                (e_drone.initial_location[1]-e_drone.delivery_location[i][1])*1.5/0.000014245)**2)
            if(curr > a):
                ans = i
                a = curr
    e_drone.visited[ans] = True
    return ans


def make_box_position():
    '''
    Purpose:
    ---
    to determine the co-ordinates of parcels

    Input Arguments:
    ---
    none

    Returns:
    ---
    none

    Example call:
    ---
    make_box_position()
    '''

    for row in range(1, 4):
        for column in range(1, 4):
            i = str(chr(ord('A')+column-1)+str(row))
            if(not i == 'A1'):
                e_drone.box[i] = [0, 0, 0]
                e_drone.box[i][0] = e_drone.box["A1"][0] + \
                    (column-1)*0.000013552
                e_drone.box[i][1] = e_drone.box["A1"][1] + (row-1)*0.000014245
                e_drone.box[i][2] = e_drone.box["A1"][2]

    for row in range(1, 4):
        for column in range(1, 4):
            i = str(chr(ord('X')+column-1)+str(row))
            if(not i == 'X1'):
                e_drone.patch[i] = [0, 0, 0]
                e_drone.patch[i][0] = e_drone.patch["X1"][0] + \
                    (column-1)*0.000013552
                e_drone.patch[i][1] = e_drone.patch["X1"][1] + \
                    (row-1)*0.000014245
                e_drone.patch[i][2] = e_drone.patch["X1"][2]


def determine_order():
    '''
    Purpose:
    ---
    to determine the order of delivery and return 

    Input Arguments:
    ---
    none

    Returns:
    ---
    none

    Example call:
    ---
    determine_order()
    '''
    for _ in range(9):
        e_drone.order.append(farthest())
        e_drone.order.append(
            nearest(e_drone.delivery_location[e_drone.order[-1]]))


def hover():
    '''
    Purpose:
    ---
    to land the drone on marker

    Input Arguments:
    ---
    none

    Returns:
    ---
    none

    Example call:
    ---
    hover()
    '''
    t = time.time()
    while time.time() - t < 0.5:
        e_drone.rpyt_cmd.rcRoll = 1500
        e_drone.rpyt_cmd.rcPitch = 1500
        e_drone.rpyt_cmd.rcThrottle = 1000
        e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)


def marker_detection():
    '''
    Purpose:
    ---
    function to detect the marker

    Input Arguments:
    ---
    none

    Returns:
    ---
    none

    Example call:
    ---
    marker_detection()
    '''

    rospy.loginfo("Initiating marker detection")
    count = 0
    count2 = 1
    x, y = 0, 0
    prev_x, prev_y = -1, -1

    while(count < 3):
        # print(e_drone.centre_x, e_drone.centre_y)
        if(e_drone.centre_x == prev_x or e_drone.centre_y == prev_y):
            # print("not detected")
            count2 += 1
        elif(prev_x != -1):
            x += e_drone.centre_x
            y += e_drone.centre_y
            count += 1

        prev_x, prev_y = e_drone.centre_x, e_drone.centre_y
        if(count2 % 3 == 0 and count == 0):
            e_drone.setpoint_location[2] = e_drone.drone_location[2] + 2.7
            t = time.time()
            while time.time() - t < 2:
                e_drone.pid(position=False, speed=100)
                # time.sleep(0.01)
            count2 = 1

        stablize_drone(time_limit=0.001, position=False, speed=100)

    x /= count
    y /= count

    # added offset of camera to y coordinate
    e_drone.setpoint_final = [
        x, y-0.0000033238333, e_drone.drone_location[2] - e_drone.vertical_distance + 0.7]
    print("marker found at ", e_drone.setpoint_final)
    reach_short_destination(height_correction=False,
                            position=False, speed=100, marker_detect=True)
    # stablize_drone(time_limit = 2, position = False, speed = 100)
    rospy.loginfo("Reached marker " + str(e_drone.current_marker_id))


def generate_csv():
    '''
    Purpose:
    ---
    to genrate the sequenced_manifest.csv file of order of deliveries and returns 

    Input Arguments:
    ---
    none

    Returns:
    ---
    none

    Example call:
    ---
    generate_csv()
    '''

    rospack = rospkg.RosPack()
    filepath = rospack.get_path('vitarana_drone') + \
        '/scripts/Task6_VD_0583_sequenced_manifest_bonus.csv'
    with open(filepath, 'w') as f:
        thewriter = csv.writer(f)
        i = 0
        while(i < len(e_drone.order)):
            box_id = e_drone.order[i]
            dest = e_drone.delivery_location[box_id]
            dest_seperated = str(dest[0]) + ';' + \
                str(dest[1]) + ';' + str(dest[2])
            thewriter.writerow(['DELIVERY', box_id, dest_seperated])
            i += 1
            box_id = e_drone.order[i]
            dest = e_drone.return_location[box_id]
            dest_seperated = str(dest[0]) + ';' + \
                str(dest[1]) + ';' + str(dest[2])
            thewriter.writerow(['RETURN', dest_seperated, box_id])
            i += 1


def main():
    '''
    Purpose:
    ---
    main function, all the task to be performed is written bellow 

    Input Arguments:
    ---
    none

    Returns:
    ---
    none

    Example call:
    ---
    main()
    '''

    make_box_position()
    determine_order()
    generate_csv()
    # box_id = 1
    i = 0
    while(i < len(e_drone.order)):
        box_id = e_drone.order[i]
        print(box_id)
        e_drone.current_marker_id = box_id
        e_drone.setpoint_final = e_drone.box[box_id]
        reach_short_destination(height_correction=4.5,
                                position=False, speed=100, marker_detect=False)

        stablize_drone(time_limit=1, position=False, speed=100)
        t = time.time()
        while time.time() - t < 0.5:
            e_drone.rpyt_cmd.rcRoll = 1500
            e_drone.rpyt_cmd.rcPitch = 1500
            e_drone.rpyt_cmd.rcThrottle = 1000
            e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)
        gripper_active(1)
        e_drone.setpoint_final = e_drone.delivery_location[box_id]
        reach_destination(height_correction=10, hc=0,
                          position=False, speed=100, delivery=True)
        # To settle on the destination
        stablize_drone(time_limit=1, position=False, speed=100)
        marker_detection()
        # hover()
        gripper_active(0)
        i += 1
        box_id = e_drone.order[i]
        print(box_id)
        e_drone.setpoint_final = e_drone.return_location[box_id]
        reach_destination(height_correction=10, hc=1,
                          position=False, speed=100, delivery=False)
        stablize_drone(time_limit=2, position=False, speed=100)
        hover()
        gripper_active(1)
        e_drone.setpoint_final = e_drone.patch[box_id]
        reach_destination(height_correction=10, hc=1,
                          position=False, speed=100, delivery=True)
        gripper_active(0)
        i += 1

    e_drone.setpoint_final = e_drone.initial_location
    reach_destination(height_correction=4, hc=1,
                      position=False, speed=100, delivery=False)
    e_drone.rpyt_cmd.rcRoll = 1500
    e_drone.rpyt_cmd.rcPitch = 1500
    e_drone.rpyt_cmd.rcThrottle = 1000
    e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)


# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    Waits for gazebo to start and then calls the main function.
if __name__ == '__main__':

    # pause of 2 sec to open and load the gazebo
    t = time.time()
    while time.time() - t < 2:
        pass

    # making e_drone object of Edrone class
    e_drone = Edrone()

    while not rospy.is_shutdown():
        main()
        break

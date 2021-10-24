#!/usr/bin/env python

'''**********************************
Purpose: Position controller

Team name : Swayamchalit Gaadi
**********************************'''

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import numpy as np
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller1')  # initializing ros node with name position_controller

        # This will contain the current location of Edrone. [latitude, longitude, altitude ]
        # this value is updating each time in gps callback function
        self.drone_location = [0.0, 0.0, 0.0]

        # This is the setpoint of location. [latitude , longitude, altitude ]
        self.setpoint_location = [19.0, 72.0, 3.0]

        # This will contain the current orientation of eDrone in quaternion format. [x,y,z,w]
        # This value is updating each time in imu callback function
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This will contain the current orientation of eDrone converted in euler angles form. [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

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
        self.latitude_Up.data = 0.000004517
        self.latitude_Low = Float32()
        self.latitude_Low.data = -0.000004517

        self.longitude_Error = Float32()
        self.longitude_Error.data = 0.0
        self.longitude_Up = Float32()
        self.longitude_Up.data = 0.0000047487
        self.longitude_Low = Float32()
        self.longitude_Low.data = -0.0000047487

        self.altitude_Error = Float32()
        self.altitude_Error.data = 0.0
        self.altitude_Up = Float32()
        self.altitude_Up.data = 0.2
        self.altitude_Low = Float32()
        self.altitude_Low.data = -0.2

        # initializing Kp, Kd and ki for [latitude, longitude, altitude] after tunning 
        self.Kp = [1080000, 1140000, 48]
        self.Ki = [0, 0, 0]
        self.Kd = [57600000, 57900000, 3000]
       
        # Declaring variable to store different error values, to be used in PID equations.
        self.change_in_error_value = [0.0, 0.0, 0.0]
        self.error_value = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.sum_error_value = [0.0, 0.0, 0.0]

        # Declaring maximum and minimum values for roll, pitch, yaw, throttle output.
        self.max_values = [2000.0, 2000.0, 2000.0, 2000.0]
        self.min_values = [1000.0, 1000.0, 1000.0, 1000.0]

        # initializing Publisher for /drone_command, /latitude_error, /longitude_error, /altitude_error and tolerences
        self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.latitude_error = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.altitude_error = rospy.Publisher('/altitude_error', Float32, queue_size=1)

        self.latitude_up = rospy.Publisher('/latitude_up', Float32, queue_size=1)
        self.longitude_up = rospy.Publisher('/longitude_up', Float32, queue_size=1)
        self.altitude_up = rospy.Publisher('/altitude_up', Float32, queue_size=1)
        self.latitude_low = rospy.Publisher('/latitude_low', Float32, queue_size=1)
        self.longitude_low = rospy.Publisher('/longitude_low', Float32, queue_size=1)
        self.altitude_low = rospy.Publisher('/altitude_low', Float32, queue_size=1)

        # Subscribing to /edrone/gps, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw {used these GUIs only to tune ;-) }
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)        # for latitude
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)      # for longitude
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)          # for altitude
    

    # Imu callback function. The function gets executed each time when imu publishes /edrone/imu/data
    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        # converting the current orientations from quaternion to euler angles 
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])


    # callback function for gps. This function gets executed each time when NavSatFix publishes /edrone/gps
    def gps_callback(self, msg):
        self.drone_location[0] = msg.latitude
        self.drone_location[1] = msg.longitude
        self.drone_location[2] = msg.altitude


    # Callback function for /pid_tuning_roll, we used it to tune latitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 600
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 30000


    # Callback function for /pid_tuning_pitch, we used it to tune longitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 600 
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 30000


    # Callback function for /pid_tuning_yaw, we used it to tune altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_yaw
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.06 
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 30
    

    # this function is containing all the pid equation to control the position of the drone
    def pid(self):
        # updating all the error values to be used in PID equation
        for i in range(3):
            self.error_value[i] = self.setpoint_location[i] - self.drone_location[i]
            self.sum_error_value[i] = self.sum_error_value[i] + self.error_value[i]
            self.change_in_error_value[i] = self.error_value[i] - self.prev_error_value[i]
            self.prev_error_value[i] = self.error_value[i]

        # assigning error value to its container to publish
        self.latitude_Error.data = self.error_value[0]
        self.longitude_Error.data = self.error_value[1]
        self.altitude_Error.data = self.error_value[2]

        # PID eqation for latitude
        output0 = self.Kp[0]*self.error_value[0] + self.Ki[0]*self.sum_error_value[0] + self.Kd[0]*self.change_in_error_value[0]
        
        # PID eqation for longitude
        output1 = self.Kp[1]*self.error_value[1] + self.Ki[1]*self.sum_error_value[1] + self.Kd[1]*self.change_in_error_value[1]
        
        # PID equation for altitude
        output2 = self.Kp[2]*self.error_value[2] + self.Ki[2]*self.sum_error_value[2] + self.Kd[2]*self.change_in_error_value[2]
        
        # updating the roll value according to PID output. 
        # {this equation will work fine when there is some yaw. to see detail opne --> https://drive.google.com/file/d/14gjse4HUIi9OoznefjOehh1HU6LUhoO7/view?usp=sharing }
        self.rpyt_cmd.rcRoll = 1500 + output0*np.cos(self.drone_orientation_euler[2]) - output1*np.sin(self.drone_orientation_euler[2])

        # updating the pitch value according to PID output
        self.rpyt_cmd.rcPitch = 1500 + output0*np.sin(self.drone_orientation_euler[2]) + output1*np.cos(self.drone_orientation_euler[2])

        # updating the throttle value according to PID output
        self.rpyt_cmd.rcThrottle = 1500 + output2
        
        # checking the boundary conditions for roll value
        if(self.rpyt_cmd.rcRoll > 1800):
            self.rpyt_cmd.rcRoll = 1800
        elif(self.rpyt_cmd.rcRoll<1200):
            self.rpyt_cmd.rcRoll = 1200

        # checking the boundary conditions for pitch value
        if(self.rpyt_cmd.rcPitch > 1800):
            self.rpyt_cmd.rcPitch = 1800
        elif(self.rpyt_cmd.rcPitch<1200):
            self.rpyt_cmd.rcPitch = 1200

        # checking the boundary conditions for throttle value
        if(self.rpyt_cmd.rcThrottle > 2000):
            self.rpyt_cmd.rcThrottle = 2000
        elif(self.rpyt_cmd.rcThrottle<1000):
            self.rpyt_cmd.rcThrottle = 1000

        
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


# main function, it will move the drone at all three points to reach the destination.
def main():
    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_location))

    # seting setpoint to first point
    e_drone.setpoint_location = [19.0, 72.0, 3]
    # running the loop until dron reaches the point under its tolerences 
    while ((e_drone.drone_location[0] > 19.0+0.000004517 or e_drone.drone_location[0] < 19.0-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 3.0+0.2 or e_drone.drone_location[2] < 3.0-0.2)):
        e_drone.pid()
        time.sleep(0.05)
    # pause of 10 sec to stablize the drone at that position
    t = time.time()
    while time.time() -t < 10:
        e_drone.pid()
        time.sleep(0.05)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))

    # seting setpoint to second point
    e_drone.setpoint_location = [19.0000451704, 72.0, 3]
    # running the loop until dron reaches the point under its tolerences 
    while ((e_drone.drone_location[0] > 19.00004517040+0.000004517 or e_drone.drone_location[0] < 19.00004517040-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 3.0+0.2 or e_drone.drone_location[2] < 3.0-0.2)):
        e_drone.pid()
        time.sleep(0.05)
    # pause of 10 sec to stablize the drone at that position
    t = time.time()
    while time.time() -t < 10:
        e_drone.pid()
        time.sleep(0.05)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))

    # seting setpoint to final point
    e_drone.setpoint_location = [19.0000451704, 72.0, 0.31]
    # running the loop until dron reaches the point under its tolerences 
    while ((e_drone.drone_location[0] > 19.00004517040+0.000004517 or e_drone.drone_location[0] < 19.00004517040-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 0.31+0.2 or e_drone.drone_location[2] < 0.31-0.2)):
        e_drone.pid()
        time.sleep(0.05)
    # pause of 10 sec to stablize the drone at that position
    t = time.time()
    while time.time() -t < 10:
        e_drone.pid()
        time.sleep(0.05)

    # turning off the drone
    e_drone.rpyt_cmd.rcRoll = 1500
    e_drone.rpyt_cmd.rcPitch = 1500
    e_drone.rpyt_cmd.rcYaw = 1500
    e_drone.rpyt_cmd.rcThrottle = 1000
    e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))
    rospy.loginfo("destination reached!!!")


if __name__ == '__main__':

    # pause of 4 sec to open and load the gazibo
    t = time.time()
    while time.time() -t < 4:
        pass

    # making e_drone object of Edrone class
    e_drone = Edrone()

    # pause of 1 sec 
    t = time.time()
    while time.time() -t < 1:
        pass

    while not rospy.is_shutdown():
        main()
        break

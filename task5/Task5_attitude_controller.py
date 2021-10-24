#!/usr/bin/env python

'''**********************************
Purpose: Attitude controller

Team name : Swayamchalit Gaadi
**********************************'''

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name attitude_controller

        # This will contain the current orientation of eDrone in quaternion format. [x,y,z,w]
        # This value is updating each time in imu callback function
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This will contain the current orientation of eDrone converted in euler angles form. [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [roll_setpoint, pitch_setpoint, yaw_setpoint], and the set_throttle value
        self.setpoint_cmd = [0.0, 0.0, 0.0]
        self.set_throttle = 0.0

        # The setpoint of orientation in euler angles at which we want to stabilize the drone
        # [roll_setpoint, pitch_psetpoint, yaw_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values.
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Declaring error values to publish for visualization in plotjuggler
        self.roll_Error = Float32()
        self.roll_Error.data = 0.0
        self.pitch_Error = Float32()
        self.pitch_Error.data = 0.0
        self.yaw_Error = Float32()
        self.yaw_Error.data = 0.0

        # initializing Kp, Kd and ki for [roll, pitch, yaw] after tunning
        self.Kp = [60   , 60   , 2100]
        self.Ki = [0    , 0    , 0]
        self.Kd = [450, 450, 435]
        
        # Declaring variable to store different error values, to be used in PID equations.
        self.error_value = [0.0, 0.0, 0.0]
        self.change_in_error_value = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.sum_error_value = [0.0, 0.0, 0.0]
        
        # Declaring maximum and minimum values for all four propelers output.
        self.max_values = [1024.0, 1024.0, 1024.0, 1024.0]
        self.min_values = [0.0, 0.0, 0.0, 0.0]

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.020  # in seconds

        # initializing Publisher for /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error = rospy.Publisher('/yaw_error', Float32, queue_size=1)

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)


    # Imu callback function. The function gets executed each time when imu publishes /edrone/imu/data
    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

     
    # drone cmd callback function. The function gets executed each time when edrone_cmd publishes /drone_command
    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.set_throttle = msg.rcThrottle


    # Callback function for /pid_tuning_roll. This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3


    # Callback function for /pid_tuning_pitch. This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3


    # Callback function for /pid_tuning_yaw. This function gets executed each time when /tune_pid publishes /pid_tuning_yaw
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 6  
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 3
    

    # this function is containing all the pid equation to control the attitude of the drone
    def pid(self):
        # converting the current orientations from quaternion to euler angles 
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
        
        # Convertng the range from 1000 to 2000 in the range of -1.575rad to 1.575rad for roll, pitch and in the range of -3.15rad to 3.15rad for yaw axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.00315 - 4.725
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.00315 - 4.725
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.00630 - 9.45

        # converting the range from 1000 to 2000 in the range of 0 to 1024 for set_throttle value
        set_throttle_value = self.set_throttle*1.024  - 1024.0
        
        # updating all the error values to be used in PID equation
        for i in range(3):
            self.error_value[i] = self.setpoint_euler[i] - self.drone_orientation_euler[i]
            self.sum_error_value[i] = self.sum_error_value[i] + self.error_value[i]
            self.change_in_error_value[i] = self.error_value[i] - self.prev_error_value[i]
            self.prev_error_value[i] = self.error_value[i]

        # assigning error values to its container to publish
        self.roll_Error.data = self.error_value[0]
        self.pitch_Error.data = self.error_value[1]
        self.yaw_Error.data = self.error_value[2]

        # PID eqation for roll
        output0 = self.Kp[0]*self.error_value[0] + self.Ki[0]*self.sum_error_value[0] + self.Kd[0]*self.change_in_error_value[0]
        
        # PID eqation for pitch
        output1 = self.Kp[1]*self.error_value[1] + self.Ki[1]*self.sum_error_value[1] + self.Kd[1]*self.change_in_error_value[1]
        
        # PID eqation for yaw
        output2 = self.Kp[2]*self.error_value[2] + self.Ki[2]*self.sum_error_value[2] + self.Kd[2]*self.change_in_error_value[2]

        # updating the prop pwm values according to PID output
        prop1 = set_throttle_value - output0 +output1 - output2
        prop2 = set_throttle_value - output0 -output1 + output2
        prop3 = set_throttle_value + output0 -output1 - output2
        prop4 = set_throttle_value + output0 +output1 + output2

        # checking the boundary conditions for prop pwm values and updating the pwm_cmd container to publish
        if(prop1>1024):
            self.pwm_cmd.prop1 = 1024
        elif(prop1<0):
            self.pwm_cmd.prop1 = 0
        else:
            self.pwm_cmd.prop1 = prop1

        if(prop2>1024):
            self.pwm_cmd.prop2 = 1024
        elif(prop2<0):
            self.pwm_cmd.prop2 = 0
        else:
            self.pwm_cmd.prop2 = prop2

        if(prop3>1024):
            self.pwm_cmd.prop3 = 1024
        elif(prop3<0):
            self.pwm_cmd.prop3 = 0
        else:
            self.pwm_cmd.prop3 = prop3

        if(prop4>1024):
            self.pwm_cmd.prop4 = 1024
        elif(prop4<0):
            self.pwm_cmd.prop4 = 0
        else:
            self.pwm_cmd.prop4 = prop4

        # publishing pwm_cmd to /edrone/pwm
        self.pwm_pub.publish(self.pwm_cmd)

        # publishing different error values
        self.roll_error.publish(self.roll_Error)
        self.pitch_error.publish(self.pitch_Error)
        self.yaw_error.publish(self.yaw_Error)


if __name__ == '__main__':

    # pause of 5 sec to open and load the gazibo
    t = time.time()
    while time.time() -t < 2:
        pass

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        # r.sleep()
        time.sleep(0.05)

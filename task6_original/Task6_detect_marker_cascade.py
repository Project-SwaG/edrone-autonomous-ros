#!/usr/bin/env python

'''
# Team name :       Swayamchalit Gaadi
# Filename:         detect_marker_cascade
# Functions:        class image_detection()
# Global variables: none
'''

from vitarana_drone.msg import MarkerData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import String,Float64,Float32, Int8
import matplotlib.pyplot as plt
import rospkg
import math
import time

class image_detection():

    # Initialise everything
    def __init__(self):
        '''
        Purpose:
        ---
        init function of image_detection class

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
        rospy.init_node('marker_detect')  # Initialise rosnode
        self.img = np.empty([])

        self.theta = 0
        self.vertical_distance = 0

        img_width = 400
        hfov_rad = 1.3962634 
        self.focal_length = (img_width/2)/math.tan(hfov_rad/2)
        self.curr_marker_id = ""
        self.marker_data = MarkerData()
        # This will contain your image frame from camera
        self.bridge = CvBridge()
       
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)

        rospy.Subscriber("/edrone/curr_marker_id", String, self.marker_id_callback)

        rospack = rospkg.RosPack()
        filepath = rospack.get_path('vitarana_drone')+'/data/cascade.xml'
        self.logo_cascade = cv2.CascadeClassifier(filepath)
        self.marker_data_pub = rospy.Publisher("/edrone/marker_data", MarkerData, queue_size = 1)

        rospy.Subscriber("/edrone/yaw", Float64, self.theta_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_callback)
        
        rospy.spin()

    def range_finder_callback(self,msg):
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
        self.range_finder_callback
        '''

        if(msg.ranges[0] < 50 and msg.ranges[0] >= 0.5):
            self.vertical_distance = msg.ranges[0]
  

    def marker_id_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for marker id

        Input Arguments:
        ---
        self
        msg : [int]
            contains the current mrker id

        Returns:
        ---
        none

        Example call:
        ---
        self.marker_id_callback
        '''
        self.curr_marker_id = msg.data


        

    def theta_callback(self, msg):
        '''
        Purpose:
        ---
        Call back fuction for yaw value

        Input Arguments:
        ---
        self
        msg : [flot]
            contains the yaw value

        Returns:
        ---
        none

        Example call:
        ---
        self.theta_callback
        '''

        self.theta = msg.data

    def pixel_to_m(self,centre_x_pixel, centre_y_pixel):
        '''
        Purpose:
        ---
        converts pixel value to meter value

        Input Arguments:
        ---
        self

        centre_x_pixel : [flot]
            pixal vlaue in x direction

        centre_y_pixel : [flot]
            pixal vlaue in y direction

        Returns:
        ---
        none

        Example call:
        ---
        pixel_to_m(self, centre_x_pixel, centre_y_pixel)
        '''
        err_x = (centre_x_pixel*self.vertical_distance)/self.focal_length
        err_y = (centre_y_pixel*self.vertical_distance)/self.focal_length
        err_x = err_x*np.cos(self.theta) - err_y*np.sin(self.theta)
        err_y= err_x*np.sin(self.theta) + err_y*np.cos(self.theta)

        self.marker_data.marker_id = 1
        self.marker_data.err_x_m = err_x
        self.marker_data.err_y_m = err_y
        self.marker_data_pub.publish(self.marker_data)


    # Callback function of camera topic
    def image_callback(self, data):
        '''
        Purpose:
        ---
        callback function oof image

        Input Arguments:
        ---
        self

        data : [array]
            contain raw image

        Returns:
        ---
        none

        Example call:
        ---
        self.image_callback()
        '''

        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
            logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05,minNeighbors=5)

            for (x, y, w, h) in logo:
                cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)


                #Pixel coordinates wrt drone
                centre_x = x + w/2 - 200
                centre_y = 200 - (y + h/2)
                self.pixel_to_m(centre_x, centre_y)

            # cv2.imshow('image',self.img)
            # cv2.waitKey(1)


        except CvBridgeError as e:
            print(e)
            return

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the image detection() class.
if __name__ == '__main__':
    try:
        image_dec_obj = image_detection()
    except rospy.ROSInterruptException:
        pass
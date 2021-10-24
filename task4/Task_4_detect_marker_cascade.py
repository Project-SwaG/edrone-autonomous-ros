#!/usr/bin/env python

'''**********************************
Purpose: marker detection

Team name : Swayamchalit Gaadi
**********************************'''

from vitarana_drone.msg import MarkerData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String,Float64,Float32, Int8
import matplotlib.pyplot as plt
import rospkg
import math
import time

class image_detection():

    # Initialise everything
    def __init__(self):
        rospy.init_node('marker_detect')  # Initialise rosnode
        self.img = np.empty([])

        self.theta = 0
        self.vertical_distance = 0

        img_width = 400
        hfov_rad = 1.3962634 
        self.focal_length = (img_width/2)/math.tan(hfov_rad/2)
        self.curr_marker_id = 0
        self.marker_data = MarkerData()
        # This will contain your image frame from camera
        self.bridge = CvBridge()
        # self.pub = rospy.Publisher('qrValue',String,queue_size=1)
        # self.pub = rospy.Publisher('/edrone/marker_data',Float64,queue_size=1)
        #self.pub = rospy.Publisher('pixValue',Int32MultiArray,queue_size=1)
        self.rate = rospy.Rate(1)
        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)

        rospy.Subscriber("/edrone/curr_marker_id", Int8, self.marker_id_callback)
        rospy.Subscriber("/edrone/vertical_distance", Float64, self.vertical_distance_callback)
        rospy.Subscriber("/edrone/yaw", Float64, self.theta_callback)


        rospack = rospkg.RosPack()
        filepath = rospack.get_path('vitarana_drone')+'/data/cascade.xml'
        self.logo_cascade = cv2.CascadeClassifier(filepath)
        self.marker_data_pub = rospy.Publisher("/edrone/marker_data", MarkerData, queue_size = 1)

        # self.err_x_m = rospy.Publisher('/edrone/err_x_m', Float64, queue_size = 1)
        # self.err_y_m = rospy.Publisher('/edrone/err_y_m', Float64, queue_size = 1)
        
        rospy.spin()


    def marker_id_callback(self, msg):
        self.curr_marker_id = msg.data

    def vertical_distance_callback(self, data):
        self.vertical_distance = data.data
        

    def theta_callback(self, msg):
        self.theta = msg.data

    def pixel_to_m(self,centre_x_pixel, centre_y_pixel):
        err_x = (centre_x_pixel*self.vertical_distance)/self.focal_length
        err_y = (centre_y_pixel*self.vertical_distance)/self.focal_length
        err_x = err_x*np.cos(self.theta) - err_y*np.sin(self.theta)
        err_y= err_x*np.sin(self.theta) + err_y*np.cos(self.theta)

        self.marker_data.marker_id = self.curr_marker_id
        self.marker_data.err_x_m = err_x
        self.marker_data.err_y_m = err_y
        self.marker_data_pub.publish(self.marker_data)
        # self.rate.sleep()

        # t = time.time()
        # while time.time() - t <= 1:
        #     pass

        # self.err_x_m.publish(err_x)
        # self.err_y_m.publish(err_y)

    # Callback function of camera topic
    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
            logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05,minNeighbors=5)

            for (x, y, w, h) in logo:
                cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                #print(x + w/2)
                #print(y + h/2)

                #Pixel coordinates wrt drone
                centre_x = x + w/2 - 200
                centre_y = 200 - (y + h/2)
                self.pixel_to_m(centre_x, centre_y)
                #data_for_publishing = Int32MultiArray(data = [centre_x, centre_y])
            
            # plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            cv2.imshow('image',self.img)
            cv2.waitKey(1)
            # plt.show()
            # plt.clf()

        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    try:
        image_dec_obj = image_detection()
    except rospy.ROSInterruptException:
        pass

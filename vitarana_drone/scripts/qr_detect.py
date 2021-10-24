#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from std_msgs.msg import String


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('qr_scanner')  # Initialise rosnode
        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)
        self.img = np.empty([])
        # This will contain your image frame from camera
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('qrValue',String,queue_size=1)
        self.rate = rospy.Rate(10)
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        rospy.spin()



    # Callback function of camera topic
    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Decoding Qrcode and publishing value
            data = ''
            decoded_data = decode(self.img)
            if decoded_data:
                for obj in decoded_data:
                    data = obj.data
                    self.pub.publish(data)
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    try:
        image_proc_obj = image_proc()
    except rospy.ROSInterruptException:
        pass

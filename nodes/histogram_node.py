#!/usr/bin/env python2

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

class HistogramNode:

    def __init__(self):
        # topic for input image
        input_img_topic = '/camera_nav/image_raw'
        self.sub_img = rospy.Subscriber(input_img_topic,
                                        Image,
                                        self.img_callback)

        # Define OpenCV bridge
        self.bridge = CvBridge()

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,
                                                 desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        print("Callback is reached")

        plt.ion()
        plt.hist(cv_image.ravel(), 256, [0, 256])
        plt.pause(0.05)
        plt.clf()

if __name__ == '__main__':
    rospy.init_node('HistogramNode', anonymous=True)
    histogram_node = HistogramNode()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        rate.sleep()

#!/usr/bin/env python2

# ROS
import rospy
from sensor_msgs.msg import Image

# OpenCV
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

# Matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation

class HistogramNode:

    def __init__(self):
        # topic for input image
        input_img_topic = '/image'
        self.sub_img = rospy.Subscriber(input_img_topic,
                                        Image,
                                        self.img_callback)

        # Define OpenCV bridge
        self.bridge = CvBridge()
        self.img_recv = False

    def img_callback(self, data):
        # Extract image from cv_bridge
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data,
                                                desired_encoding="passthrough")

            if self.img_recv is False:
                self.img_recv = True
        except CvBridgeError as e:
            print(e)

    def animate(self, num):
        # Plot features
        plt.cla()
        plt.title("Image Intensity Histogram")
        plt.xlabel("Intensity")
        plt.ylabel("Count")
        plt.xlim(0, 255)

        if self.img_recv is True:
            plt.hist(self.cv_image.ravel(), 256, [0, 256])

    def plotter(self):
        while not rospy.is_shutdown():
            fig = plt.figure()
            ani = animation.FuncAnimation(fig,
                                      self.animate,
                                      frames=None,
                                      init_func=None,
                                      interval=25)
            plt.show()
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('HistogramNode', anonymous=True)
    histogram_node = HistogramNode()
    histogram_node.plotter()

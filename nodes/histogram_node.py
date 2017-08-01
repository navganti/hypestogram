#!/usr/bin/env python2

# ROS
import rospy
from sensor_msgs.msg import Image

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation

# Numpy
import numpy as np

# Argparse
import argparse


class HistogramNode:

    def __init__(self, args):
        # topic for input image and ROS subscriber
        self.input_img_topic = '/image'
        self.sub_img = rospy.Subscriber(self.input_img_topic,
                                        Image,
                                        self.img_callback)

        # Define OpenCV bridge and variables
        self.bridge = CvBridge()
        self.cv_image = None

        # Set flags and mode
        self.img_recv = False
        self.name = args.name
        self.text_only = args.t
        self.text_plot = args.tp

        # Set thresholds for a washed out or dark image
        self.high_index = 250
        self.low_index = 30
        self.high_bin_threshold = 15000
        self.low_bin_threshold = 15000
        self.flag = False

        # Count variable - ensures that the text analysis waits a few cycles
        # prior to printing out the verdict
        self.img_count = 0

    def img_callback(self, data):
        # Extract image from cv_bridge
        try:
            image = self.bridge.imgmsg_to_cv2(data,
                                                desired_encoding="passthrough")

            # Convert to grayscale
            self.cv_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

            if self.img_recv is False:
                self.img_recv = True

            if self.text_only is True or self.text_plot is True:
                self.histogram_analysis()

            self.img_count += 1

        except CvBridgeError as e:
            print(e)

    # Plots the histogram of the current image once. Only called if -tp is set.
    def offline_plot(self):
        fig = plt.figure()
        plt.cla()
        plt.title(self.name)
        plt.xlabel("Intensity")
        plt.ylabel("Count")
        plt.xlim(0, 255)
        plt.hist(self.cv_image.ravel(), 256, [0, 256])
        plt.show()

    # Provides a text output with an analysis of the current image. Only called
    # if -tp or -t are set.
    def histogram_analysis(self):
        if self.img_recv is True:
            # Extract histogram info
            hist = np.bincount(self.cv_image.ravel(), minlength=256)

            # Get the high subset of this bincount, and calculate the maximum #
            high_subset = hist[self.high_index:255]
            max_high = high_subset.max()

            # Get the low subset
            low_subset = hist[0:self.low_index]
            max_low = low_subset.max()

            # Get the mid subset
            mid_subset = hist[self.low_index + 1: self.high_index - 1]
            max_mid = mid_subset.max()

            if self.img_count == 5:
                if max_high >= max_mid:
                    print "Image", self.name, "is washed out"
                    self.flag = True
                elif max_high > self.high_bin_threshold:
                    print "Image", self.name, "has portions of the" \
                                              " image that are washed out."
                    self.flag = True

                if max_low >= max_mid:
                    print "Image", self.name, "is too dark."
                    self.flag = True
                elif max_low >self.low_bin_threshold:
                    print "Image", self.name, "has portions of the" \
                                        " image that are too dark."
                    self.flag = True

                if self.flag is False:
                    print "Image", self.name, "is OK."

                if self.text_plot is True:
                    self.offline_plot()

    # Animation function for live histogram visualization. Only called if
    # -t or -tp are not set.
    def animate(self, num):
        # Plot characteristics
        plt.cla()
        plt.title(self.name)
        plt.xlabel("Intensity")
        plt.ylabel("Count")
        plt.xlim(0, 255)

        if self.img_recv is True:
            plt.hist(self.cv_image.ravel(), 256, [0, 256])

    # Main plotter function for live visualization.
    def run(self):
        while not rospy.is_shutdown():
            if self.text_only is False and self.text_plot is False:
                fig = plt.figure()
                ani = animation.FuncAnimation(fig, self.animate, frames=None,
                                              init_func=None, interval=25)
                plt.show()
            rospy.spin()


if __name__ == '__main__':
    # Parse mode argument - determine whether to run in online or offline mode
    parser = argparse.ArgumentParser(
        description='Plot an image intensity histogram of a ros topic.')
    parser.add_argument(
        "--name", type=str, default="Image Intensity Histogram",
        help="If the optional parameter is added, run in offline mode.")
    parser.add_argument("-t", action='store_true', default=False,
                        help="Does not visualize histogram and provides text"
                             " feedback on whether or not an image is washed"
                             " out or too dark.")
    parser.add_argument("-tp", action='store_true', default=False,
                        help="Visualizes histogram once and provides text"
                             " feedback on whether or not an image is washed"
                             " out or too dark.")

    # rospy argument required to parse roslaunch topics.
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('HistogramNode', anonymous=True)
    histogram_node = HistogramNode(args)
    histogram_node.run()

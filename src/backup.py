#!/usr/bin/env python

from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import rospy
import cv2
import sys
import time
import roslib
import numpy as np
import rospkg
import datetime


class Backup:
    # Global variables
    package = "multispectral_processing"
    file_backup = "/data/backup/single_frame/"
    file_stream = "/data/backup/multi_frames/"
    OPERATION = False
    counter = 0

    def __init__(self):
        # Optimization in OpenCV
        cv2.useOptimized()
        # Initialize cv bridge
        self.bridge = CvBridge()
        # Subcribers
        self.sub_1_ = message_filters.Subscriber("/camera/image_raw", Image)
        self.sub_2_ = message_filters.Subscriber(
            "/band/3/interpolated/image_raw", Image)
        self.sub_3_ = message_filters.Subscriber("/band/ndvi/image_raw", Image)
        self.sub_4_ = message_filters.Subscriber(
            "/band/ndvi_colored/image_raw", Image)
        self.sub_5_ = message_filters.Subscriber(
            "/kinect2/hd/image_color_rect", Image)
        self.sub_6_ = message_filters.Subscriber(
            "/kinect2/hd/image_depth_rect", Image)
        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_1_, self.sub_2_, self.sub_3_, self.sub_4_, self.sub_5_, self.sub_6_], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        # Get an instance of rospack with the default search paths
        rospack = rospkg.RosPack()
        # List all packages
        rospack.list()
        self.path_backup = rospack.get_path(self.package) + self.file_backup
        self.path_stream = rospack.get_path(self.package) + self.file_stream
        # Display info
        rospy.loginfo("Backup node (python) started")
        print("Press esc to save single frame, S or s to save stream of frames.")
        print("Files are saved in " + self.path_backup +
              " and " + self.path_stream + " folders.")

    def callback(self, in1, in2, in3, in4, in5, in6):
        # Convert from ROS to OpenCV
        try:
            img_1 = self.bridge.imgmsg_to_cv2(
                in1, desired_encoding="passthrough")
            img_2 = self.bridge.imgmsg_to_cv2(
                in2, desired_encoding="passthrough")
            img_3 = self.bridge.imgmsg_to_cv2(
                in3, desired_encoding="passthrough")
            img_4 = self.bridge.imgmsg_to_cv2(
                in4, desired_encoding="passthrough")
            img_5 = self.bridge.imgmsg_to_cv2(
                in5, desired_encoding="passthrough")
            img_6 = self.bridge.imgmsg_to_cv2(
                in6, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        # Display images
        cv2.namedWindow("Image 1", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image 1", 600, 600)
        cv2.imshow("Image 1", img_1)

        cv2.namedWindow("Image 2", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image 2", 600, 600)
        cv2.imshow("Image 2", img_2)

        cv2.namedWindow("Image 3", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image 3", 600, 600)
        cv2.imshow("Image 3", img_3)

        cv2.namedWindow("Image 4", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image 4", 600, 600)
        cv2.imshow("Image 4", img_4)

        cv2.namedWindow("Image 5", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image 5", 600, 600)
        cv2.imshow("Image 5", img_5)

        cv2.namedWindow("Image 6", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image 6", 600, 600)
        cv2.imshow("Image 6", img_6)

        # Key listener
        # Save single frame with <esc> key, save stream of frames with <s, S> key
        key = cv2.waitKey(3)
        if(key == 27):
            self.OPERATION = False
            self.backupImages(img_1, img_2, img_3, img_4,
                              img_5, img_6, self.path_backup)
            rospy.loginfo("Single frame saved.")
        elif (key == 83 or key == 115):
            # Enable or disable the stream saving procedure
            if (self.OPERATION):
                rospy.loginfo("Saving stream of frames disabled.")
                self.OPERATION = False
            else:
                rospy.loginfo("Saving stream of frames enabled.")
                self.OPERATION = True
        if self.OPERATION:
            self.backupImages(img_1, img_2, img_3, img_4,
                              img_5, img_6, self.path_stream)

    # Save the backup of the images
    def backupImages(self, img_1, img_2, img_3, img_4, img_5, img_6, path):
        # Current date/time based on current system
        x = datetime.datetime.now()
        # Titles of images
        s1 = ""
        s2 = ""
        s3 = ""
        s4 = ""
        s5 = ""
        s6 = ""
        if self.OPERATION:
            s1 = path + str(self.counter) + "_multispectral_camera.png"
            s2 = path + str(self.counter) + "_band3_interpolated.png"
            s3 = path + str(self.counter) + "_multispectral_ndvi.png"
            s4 = path + str(self.counter) + "_multispectral_ndvi_colored.png"
            s5 = path + str(self.counter) + "_kinect_hd_rgb.png"
            s6 = path + str(self.counter) + "_kinect_hd_depth.png"
            self.counter = self.counter + 1
        else:
            s1 = path + str(x.year) + str(x.month) + \
                str(x.day) + "_multispectral_camera.png"
            s2 = path + str(x.year) + str(x.month) + \
                str(x.day) + "_band3_interpolated.png"
            s3 = path + str(x.year) + str(x.month) + \
                str(x.day) + "_multispectral_ndvi.png"
            s4 = path + str(x.year) + str(x.month) + \
                str(x.day) + "_multispectral_ndvi_colored.png"
            s5 = path + str(x.year) + str(x.month) + \
                str(x.day) + "_kinect_hd_rgb.png"
            s6 = path + str(x.year) + str(x.month) + \
                str(x.day) + "_kinect_hd_depth.png"
        # Save images
        try:
            cv2.imwrite(s1, img_1)
            cv2.imwrite(s2, img_2)
            cv2.imwrite(s3, img_3)
            cv2.imwrite(s4, img_4)
            cv2.imwrite(s5, img_5)
            cv2.imwrite(s6, img_6)
        except cv2.error:
            rospy.logerr(
                "Unable to save the images. The node will be shutted down.")
            rospy.signal_shutdown("Exit")


def main(args):
    rospy.init_node("backup", anonymous=True)
    Backup()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    rospy.signal_shutdown("Exit")
    rospy.loginfo("Backup node (python) shutted down")


if __name__ == "__main__":
    main(sys.argv)

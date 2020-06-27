#!/usr/bin/env python

from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import message_filters
import rospy
import cv2
import sys
import time
import roslib
import numpy as np

# Experimental node for synchronization and stereo image processing


class Synchronizer:
    # Global variables
    BAND_WIDTH = 960
    BAND_HEIGHT = 540
    D_MODEL = "plumb_bob"
    FRAME_ID = "stereo_frame"
    IME_B = "mono8"

    def __init__(self):
        # Optimization in OpenCV
        cv2.useOptimized()
        # Initialize cv bridge
        self.bridge = CvBridge()
        # Subcribers
        self.sub_1_ = message_filters.Subscriber("/band/1/image_raw", Image)
        self.sub_2_ = message_filters.Subscriber(
            "/kinect2/qhd/image_color_rect", Image)
        self.sub_1_i = message_filters.Subscriber(
            "/band/camera_info", CameraInfo)
        self.sub_2_i = message_filters.Subscriber(
            "/kinect2/qhd/camera_info", CameraInfo)
        # Publishers
        self.pub_1_ = rospy.Publisher(
            "/stereo/right/image_raw", Image, queue_size=1)
        self.pub_2_ = rospy.Publisher(
            "/stereo/left/image_raw", Image, queue_size=1)
        self.pub_1_i = rospy.Publisher(
            "/stereo/right/camera_info", CameraInfo, queue_size=1)
        self.pub_2_i = rospy.Publisher(
            "/stereo/left/camera_info", CameraInfo, queue_size=1)
        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_1_, self.sub_2_, self.sub_1_i, self.sub_2_i], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        # Display info
        rospy.loginfo("Synchronizer node (python) started")
        # Init camera info
        self.M_CAMERA_INFO = CameraInfo()
        self.M_CAMERA_INFO.header.seq = 0
        self.M_CAMERA_INFO.header.frame_id = self.FRAME_ID
        self.M_CAMERA_INFO.width = self.BAND_WIDTH
        self.M_CAMERA_INFO.height = self.BAND_HEIGHT
        self.M_CAMERA_INFO.distortion_model = self.D_MODEL

        self.K_CAMERA_INFO = CameraInfo()
        self.K_CAMERA_INFO.header.frame_id = self.FRAME_ID
        self.K_CAMERA_INFO.header.seq = 0
        self.K_CAMERA_INFO.width = self.BAND_WIDTH
        self.K_CAMERA_INFO.height = self.BAND_HEIGHT
        self.K_CAMERA_INFO.distortion_model = self.D_MODEL

    def callback(self, in1, in2, in3, in4):
        # Set to the camera info the time stamp
        self.M_CAMERA_INFO.header.stamp = rospy.Time.from_sec(time.time())
        self.K_CAMERA_INFO.header.stamp = rospy.Time.from_sec(time.time())

        # Convert from ROS to OpenCV
        try:
            img1 = self.bridge.imgmsg_to_cv2(in1, self.IME_B)
            img2 = self.bridge.imgmsg_to_cv2(in2, self.IME_B)
        except CvBridgeError as e:
            print(e)

        # Set camera info
        self.M_CAMERA_INFO.D = in3.D
        self.M_CAMERA_INFO.K = in3.K
        self.M_CAMERA_INFO.R = in3.R
        self.M_CAMERA_INFO.P = in3.P

        self.K_CAMERA_INFO.D = in4.D
        self.K_CAMERA_INFO.K = in4.K
        self.K_CAMERA_INFO.R = in4.R
        self.K_CAMERA_INFO.P = in4.P

        # Resize image
        img_rm = self.resizeImage(img1)

        # Publish images
        try:
            pim1 = self.bridge.cv2_to_imgmsg(img_rm, encoding=self.IME_B)
            pim1.header = self.M_CAMERA_INFO.header

            pim2 = self.bridge.cv2_to_imgmsg(img2, encoding=self.IME_B)
            pim2.header = self.M_CAMERA_INFO.header

            # Publish image bands topics
            self.pub_1_.publish(pim1)
            self.pub_1_i.publish(self.M_CAMERA_INFO)
            self.pub_2_.publish(pim2)
            self.pub_2_i.publish(self.K_CAMERA_INFO)
        except CvBridgeError as e:
            print(e)

    def resizeImage(self, img):
        # Bicubic Interpolation for reseizing
        rimg = cv2.resize(img, (960, 764), interpolation=cv2.INTER_CUBIC)
        # Crop images to fit with depth images 960x540
        return rimg[112:rimg.shape[0]-112, :]

    def stop(self):
        self.sub_1_.unregister()
        self.sub_1_i.unregister()
        self.sub_2_.unregister()
        self.sub_2_i.unregister()
        self.pub_1_.unregister()
        self.pub_1_i.unregister()
        self.pub_2_.unregister()
        self.pub_2_i.unregister()
        rospy.signal_shutdown("Exit")
        rospy.loginfo("Synchronizer node (python) shutted down")


def main(args):
    rospy.init_node("synchronizer", anonymous=True)
    synchronizer = Synchronizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    synchronizer.stop()


if __name__ == "__main__":
    main(sys.argv)

#!/usr/bin/env python

from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
import message_filters
import rospy
import sys
import time
import roslib
import numpy as np


class StereoCalibrator:
    # Termination criteria for point detection
    RAW_WIDTH = 1280
    RAW_HEIGHT = 1024
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    patternsize = (7, 5)
    imagesize = (1280, 1024)
    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((7*5, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints = []    # 3d point in real world space
    imgpoints0 = []   # 2d points in image plane
    imgpoints1 = []   # 2d points in image plane
    numImages = 50   # Number of captured images for calibration
    mtx0 = []         # Camera matrix
    dist0 = []        # Camera distortion
    mtx1 = []         # Camera matrix
    dist1 = []        # Camera distortion
    # Button triggers
    calibrationReady = False
    calibration = False
    display1 = False
    display2 = False
    display3 = False
    # Cube coordinates
    axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
    # 8 bit image type for publishing
    IME_B = "mono8"

    def __init__(self):
        # Optimization in OpenCV
        cv2.useOptimized()
        # Initialize cv bridge
        self.bridge = CvBridge()
        # Initialize cv bridge
        self.bridge = CvBridge()
        # Subcribers
        self.sub_1_ = message_filters.Subscriber(
            "/stereo/right/image_raw", Image)
        self.sub_2_ = message_filters.Subscriber(
            "/stereo/left/image_raw", Image)
        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_1_, self.sub_2_], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        # Display info
        rospy.loginfo("Stereo calibrator node (python) started")
        print("Calibrator options are:\n   esc - exit\n   c - points capturing and calibration\n   u - undistort image method 1\n   r - undistort image method 2\n   p - points pose estimation")

    def callback(self, img1, img2):
        # The callback processing the pairs of numbers that arrived at approximately the same time
        try:
            imr = self.bridge.imgmsg_to_cv2(img1, self.IME_B)
            iml = self.bridge.imgmsg_to_cv2(img2, self.IME_B)
        except CvBridgeError as e:
            print(e)

        key = cv2.waitKey(3)
        self.calibrate(iml, imr, key)

    def calibrate(self, img0, img1, key):
        # Key actions
        if key == 27:
            self.stop()
        elif key == 99:
            if self.calibration:
                print("Calibration points capturing process is off.")
                self.calibration = False
            else:
                print("Calibration points capturing process is on.")
                self.calibration = True
        elif key == 117:
            if self.display1:
                print("Calibrated image using cv2.undistort() is off.")
                cv2.destroyWindow("Calibrated image using cv2.undistort()")
                self.display1 = False
            else:
                print("Calibrated image using cv2.undistort() is on.")
                self.display1 = True
        elif key == 114:
            if self.display2:
                print("Calibrated image using remapping is off.")
                cv2.destroyWindow("Calibrated image using remapping")
                self.display2 = False
            else:
                print("Calibrated image using remapping is on.")
                self.display2 = True
        elif key == 112:
            if self.display3:
                print("Point esstimation is off.")
                self.display3 = False
            else:
                print("Point esstimation is on.")
                self.display3 = True

        # For grayscale if not just convert them
        gray0 = img0
        gray1 = img1

        # Find the chess board corners
        ret0, corners0 = cv2.findChessboardCorners(
            gray0, self.patternsize, None, cv2.CALIB_CB_FAST_CHECK)
        ret1, corners1 = cv2.findChessboardCorners(
            gray1, self.patternsize, None, cv2.CALIB_CB_FAST_CHECK)

        # If found, add object points, image points (after refining them)
        if ret0 and ret1:
            # Finds the positions of internal corners of the chessboard
            corn0 = cv2.cornerSubPix(
                gray0, corners0, (11, 11), (-1, -1), self.criteria)
            corn1 = cv2.cornerSubPix(
                gray1, corners1, (11, 11), (-1, -1), self.criteria)

            if self.calibration:
                self.objpoints.append(self.objp)
                self.imgpoints0.append(corn0)
                self.imgpoints1.append(corn1)

                # Capture images and then stop
                if (len(self.imgpoints0) == self.numImages) and (len(self.imgpoints1) == self.numImages):
                    self.calibrationReady = True

            # Draw and display the corners
            img0 = cv2.drawChessboardCorners(
                img0, self.patternsize, corn0, ret0)
            img1 = cv2.drawChessboardCorners(
                img1, self.patternsize, corn1, ret1)

        # Calibration process
        if self.calibrationReady:
            # Disable points capturing
            self.calibration = False
            cv2.destroyWindow("Left camera")
            cv2.destroyWindow("Right camera")
            self.mtx0 = []
            self.dist0 = []
            self.mtx1 = []
            self.dist1 = []
            self.R = []
            self.T = []

            # Calibration process
            print("Calibration started. Please wait...")
            (_, self.mtx0, self.dist0, self.mtx1, self.dist1, self.R, self.T, self.E, _) = cv2.stereoCalibrate(
                self.objpoints, self.imgpoints0, self.imgpoints1, np.array(self.mtx0), np.array(self.dist0), np.array(self.mtx1), np.array(self.dist1), self.imagesize, None, flags=cv2.CALIB_FIX_INTRINSIC)

            print(self.T)
            print("----------------------------")
            print(self.R)
            print("Calibration finished...")

            # Init for next points capturing
            self.objpoints = []
            self.imgpoints0 = []
            self.imgpoints1 = []
            self.calibrationReady = False

        if self.display3:
            self.R1, self.R2, self.P1, self.P2, _, _, _ = cv2.stereoRectify(
                self.mtx0, self.dist0, self.mtx1, self.dist1, self.imagesize, self.R, self.T, None, None, None, None, None, flags=0, alpha=-1, newImageSize=self.imagesize)

            self.M01, self.M02 = cv2.initUndistortRectifyMap(
                self.mtx0, self.dist0, self.R1, self.P1, self.imagesize, cv2.CV_32FC1)
            self.M11, self.M12 = cv2.initUndistortRectifyMap(
                self.mtx1, self.dist1, self.R2, self.P2, self.imagesize, cv2.CV_32FC1)

            fixedLeft = cv2.remap(img0, self.M01, self.M02,
                                  interpolation=cv2.INTER_LINEAR)
            fixedRight = cv2.remap(
                img1, self.M11, self.M12, interpolation=cv2.INTER_LINEAR)

            cv2.namedWindow("ll", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ll", 600, 600)
            cv2.imshow("ll", fixedLeft)

            cv2.namedWindow("rr", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("rr", 600, 600)
            cv2.imshow("rr", fixedRight)

        if self.display2:
            corn0 = cv2.cornerSubPix(
                gray0, corners0, (11, 11), (-1, -1), self.criteria)
            axis = np.float32(
                [[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
            imgpts, _ = cv2.projectPoints(
                axis, self.R, self.T, self.mtx1, self.dist1)
            print(imgpts)

        if self.display1:
            rows, cols = img1.shape
            dst = cv2.warpPerspective(img0, self.E, (self.RAW_WIDTH, self.RAW_HEIGHT))
            # M = self.R[0:2,:]
            # dst = cv2.warpAffine(img1, M, (cols, rows))
            cv2.namedWindow("Rotated", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Rotated", 600, 600)
            cv2.imshow("Rotated", dst)

        cv2.namedWindow("Left camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Left camera", 600, 600)
        cv2.imshow("Left camera", img0)

        cv2.namedWindow("Right camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Right camera", 600, 600)
        cv2.imshow("Right camera", img1)

    def stop(self):
        rospy.signal_shutdown("Exit")
        rospy.loginfo("Stereo calibrator node (python) shutted down")


def main(args):
    rospy.init_node("stereo_calibrator", anonymous=True)
    stereoCalibrator = StereoCalibrator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    stereoCalibrator.stop()


if __name__ == "__main__":
    main(sys.argv)

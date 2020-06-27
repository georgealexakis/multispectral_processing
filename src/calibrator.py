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


class Calibrator:
    # Termination criteria for point detection
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    patternsize = (7, 5)
    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((7*5, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints = []   # 3d point in real world space
    imgpoints = []   # 2d points in image plane
    numImages = 100  # Number of captured images for calibration
    mtx = []         # Camera matrix
    dist = []        # Camera distortion
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
        # Subcribers
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        # Display info
        rospy.loginfo("Calibrator node (python) started")
        print("Calibrator options are:\n   esc - exit\n   c - points capturing and calibration\n   u - undistort image method 1\n   r - undistort image method 2\n   p - points pose estimation")

    def callback(self, img_i):
        # The callback processing the pairs of numbers that arrived at approximately the same time
        try:
            img = self.bridge.imgmsg_to_cv2(img_i, self.IME_B)
        except CvBridgeError as e:
            print(e)

        key = cv2.waitKey(3)
        self.calibrate(img, key)

    def draw(self, img, corners, imgpts):
        corner0 = tuple(corners[0].ravel())
        img = cv2.line(img, corner0, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner0, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner0, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return img

    def calibrate(self, img, key):
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

        # For grayscale
        gray = img

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(
            gray, self.patternsize, None, cv2.CALIB_CB_FAST_CHECK)

        # If found, add object points, image points (after refining them)
        if ret:
            # Finds the positions of internal corners of the chessboard
            corn = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.criteria)

            if self.calibration:
                self.objpoints.append(self.objp)
                self.imgpoints.append(corn)

                # Capture images and then stop
                if (len(self.imgpoints) == self.numImages):
                    self.calibrationReady = True

            # Draw and display the corners
            img = cv2.drawChessboardCorners(
                img, self.patternsize, corn, ret)

        # Calibration process
        if self.calibrationReady:
            # Disable points capturing
            self.calibration = False
            cv2.destroyWindow("Calibration points")
            self.mtx = []
            self.dist = []

            # Calibration process
            print("Calibration started. Please wait...")
            ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, gray.shape[::-1], None, None)

            # Display results
            print("------------Camera matrix----------")
            print(self.mtx)
            print("-----------Distortion coefficients-----------")
            print(self.dist)
            print("-----------Rotation vectors-----------")
            print(self.rvecs)
            print("-----------Transformation vectors-----------")
            print(self.tvecs)
            print("----------------------")

            # Re-projection error calculation
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(
                    self.objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.dist)
                error = cv2.norm(
                    self.imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error

            print("Total mean re-projection error: " +
                  str(mean_error/len(self.objpoints)))
            print("----------------------")
            print("Calibration finished...")

            # Init for next points capturing
            self.objpoints = []
            self.imgpoints = []
            self.calibrationReady = False

        # Display calibrated image using cv2.undistort()
        if self.display1 and (len(self.mtx) > 0):
            h, w = img.shape[:2]
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w, h), 1, (w, h))
            dst = cv2.undistort(
                img, self.mtx, self.dist, None, newcameramtx)
            cv2.namedWindow(
                "Calibrated image using cv2.undistort()", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                "Calibrated image using cv2.undistort()", 600, 600)
            cv2.imshow("Calibrated image using cv2.undistort()", dst)

        # Display calibrated image using remapping
        if self.display2 and (len(self.mtx) > 0):
            h, w = img.shape[:2]
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w, h), 1, (w, h))
            mapx, mapy = cv2.initUndistortRectifyMap(
                self.mtx, self.dist, None, newcameramtx, (w, h), 5)
            dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
            cv2.namedWindow(
                "Calibrated image using remapping", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Calibrated image using remapping", 600, 600)
            cv2.imshow("Calibrated image using remapping", dst)

        # Display pose estimation
        if self.display3 and (len(self.mtx) > 0):
            if ret == True:
                # Finds the positions of internal corners of the chessboard
                corn = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self.criteria)
                # Find the rotation and translation vectors
                _, rvec, tvec, _ = cv2.solvePnPRansac(
                    self.objp, corn, self.mtx, self.dist)
                # Project 3D points to image plane
                imgpts, _ = cv2.projectPoints(
                    self.axis, rvec, tvec, self.mtx, self.dist)
                # Draw lines
                img = self.draw(img, corn, imgpts)

        cv2.namedWindow("Calibration points", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Calibration points", 600, 600)
        cv2.imshow("Calibration points", img)

    def stop(self):
        self.sub.unregister()
        rospy.signal_shutdown("Exit")
        rospy.loginfo("Calibrator node (python) shutted down")


def main(args):
    rospy.init_node("calibrator", anonymous=True)
    calibrator = Calibrator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    calibrator.stop()


if __name__ == "__main__":
    main(sys.argv)

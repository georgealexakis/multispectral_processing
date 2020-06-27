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
import copy
import roslib
import numpy as np
import rospkg


class FeaturesRegistrator:
    # Image parameters
    RAW_WIDTH = 1278
    RAW_HEIGHT = 1017
    D_MODEL = "plumb_bob"
    FRAME_ID = "multispectral_frame"
    IME_B = "mono8"
    IME_C = "bgr8"
    package = "multispectral_processing"
    file = "/resources/homography1.yaml"
    HOMOGRAPHY_MATRIX = []
    # Number of matches
    MAX_FEATURES = 1000
    # Lower is more accurate (higher takes more features)
    GOOD_MATCH_PERCENT = 0.15
    # Node operation capturing or not
    OPERATION = False
    # Capture best homography matrix
    BEST_HOMOGRAPHY = []
    MAX_MATCHES = 0
    MIN_DIFF = 255

    def __init__(self, args):
        # Optimization in OpenCV
        cv2.useOptimized()
        # Initialize cv bridge
        self.bridge = CvBridge()
        # Subcribers
        self.sub_1_ = message_filters.Subscriber(
            "/band/3/interpolated/image_raw", Image)
        self.sub_2_ = message_filters.Subscriber(
            "/kinect2/hd/image_color_rect", Image)
        self.sub_3_ = message_filters.Subscriber(
            "/kinect2/hd/image_depth_rect", Image)
        # Publishers
        self.pub_1_ = rospy.Publisher(
            "/multispectral/image_color", Image, queue_size=1)
        self.pub_2_ = rospy.Publisher(
            "/multispectral/image_mono", Image, queue_size=1)
        self.pub_3_ = rospy.Publisher(
            "/multispectral/image_depth", Image, queue_size=1)
        self.pub_4_ = rospy.Publisher(
            "/multispectral/camera_info", CameraInfo, queue_size=1)
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_1_, self.sub_2_, self.sub_3_], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        # Get an instance of rospack with the default search paths
        rospack = rospkg.RosPack()
        # List all packages
        rospack.list()
        # Get package path
        self.path = rospack.get_path(self.package) + self.file
        # Read camera parameters
        self.readCameraParameters()
        # Display info
        rospy.loginfo("Feature matcher node started")
        if((len(args) > 1) and (args[1] == "capture")):
            self.OPERATION = True
            rospy.loginfo(
                "Perspective transformation matrix capturing mode started.")
        else:
            rospy.loginfo("Publishing mode started.")

    def callback(self, img1, img2, img3):
        # The callback processing the pairs of numbers that arrived at approximately the same time
        try:
            im1 = self.bridge.imgmsg_to_cv2(
                img1, self.IME_B)
            im2BGR = self.bridge.imgmsg_to_cv2(
                img2, self.IME_C)
            im3 = self.bridge.imgmsg_to_cv2(
                img3, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        # BGR channel splitting and acquire only RED channel
        im2 = copy.deepcopy(im2BGR[:, :, 2])
        # Key listener
        key = cv2.waitKey(3)
        # Operations
        if(self.OPERATION):
            # Reset captures with r or R
            if (key == 114) or (key == 82):
                self.BEST_HOMOGRAPHY = []
                self.MAX_MATCHES = 0
                self.MIN_DIFF = 255
            # The estimated homography will be calculated
            h = self.computeHomographyFeatures(im1, im2)
            # Check if homography matrix exists
            if(h is None):
                r1 = np.zeros((self.RAW_HEIGHT, self.RAW_WIDTH, 3), np.uint8)
                r2 = np.zeros((self.RAW_HEIGHT, self.RAW_WIDTH), np.uint8)
            else:
                r1 = cv2.warpPerspective(im2BGR, np.asarray(
                    h), (self.RAW_WIDTH, self.RAW_HEIGHT))
                r2 = cv2.warpPerspective(im3, np.asarray(
                    h), (self.RAW_WIDTH, self.RAW_HEIGHT))
            if(len(self.BEST_HOMOGRAPHY) == 0):
                r3 = np.zeros((self.RAW_HEIGHT, self.RAW_WIDTH, 3), np.uint8)
                r4 = np.zeros((self.RAW_HEIGHT, self.RAW_WIDTH), np.uint8)
            else:
                r3 = cv2.warpPerspective(im2BGR, np.asarray(
                    self.BEST_HOMOGRAPHY), (self.RAW_WIDTH, self.RAW_HEIGHT))
                r4 = cv2.warpPerspective(im3, np.asarray(
                    self.BEST_HOMOGRAPHY), (self.RAW_WIDTH, self.RAW_HEIGHT))
                # Press esc to save matrix only the best result
                if key == 27:
                    self.saveHomography()
            vis1 = np.concatenate((r1, r3), axis=1)
            vis2 = np.concatenate((r2, r4), axis=1)
            # Display aligned images
            cv2.namedWindow(
                "Aligned kinect RGB image (Real time - Best result)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                "Aligned kinect RGB image (Real time - Best result)", 600, 600)
            cv2.imshow(
                "Aligned kinect RGB image (Real time - Best result)", vis1)

            cv2.namedWindow(
                "Aligned kinect depth image (Real time - Best result)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                "Aligned kinect depth image (Real time - Best result)", 600, 600)
            cv2.imshow(
                "Aligned kinect depth image (Real time - Best result)", vis2)
        else:
            cv2.destroyWindow("Aligned kinect RGB image")
            # Applies perspective transformation to the images
            depth = cv2.warpPerspective(im3, np.asarray(
                self.HOMOGRAPHY_MATRIX), (self.RAW_WIDTH, self.RAW_HEIGHT))
            rgb = cv2.warpPerspective(im2BGR, np.asarray(
                self.HOMOGRAPHY_MATRIX), (self.RAW_WIDTH, self.RAW_HEIGHT))
            # Publish topics
            self.publishImages(im1, depth, rgb)

    # Find homography matrix with feature matching. More: https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
    def computeHomographyFeatures(self, img1, img2):
        # Detect ORB features and compute descriptors
        orb = cv2.ORB_create(self.MAX_FEATURES)
        keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
        keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

        # Match features with Brute-Force Matcher
        matcher = cv2.DescriptorMatcher_create(
            cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
        matches = matcher.match(descriptors1, descriptors2, None)

        # Sort matches by DMatch.distance (Distance between descriptors. The lower, the better it is.)
        matches.sort(key=lambda x: x.distance, reverse=False)

        # Remove not so good matches
        numGoodMatches = int(len(matches) * self.GOOD_MATCH_PERCENT)
        matches = matches[:numGoodMatches]

        # Draw top matches 1/2
        imMatches = cv2.drawMatches(
            img1, keypoints1, img2, keypoints2, matches, None)
        cv2.putText(imMatches, "Detected features (left image): " + str(len(keypoints1)),
                    (10, 800), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(imMatches, "Detected features (right image): " + str(len(keypoints2)),
                    (10, 850), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        # Set a threshold of acceptable number of features
        if (numGoodMatches >= (self.MAX_FEATURES*0.04)):
            # Draw top matches 2/2
            cv2.putText(imMatches, "Matches between the images: " + str(numGoodMatches),
                        (10, 900), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(imMatches, "Matches with min difference: " + str(self.MAX_MATCHES),
                        (10, 950), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(imMatches, "Min captured difference: " + str(self.MIN_DIFF),
                        (10, 1000), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.namedWindow("Matches", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Matches", 600, 600)
            cv2.imshow("Matches", imMatches)

            # Extract location of good matches
            points1 = np.zeros((len(matches), 2), dtype=np.float32)
            points2 = np.zeros((len(matches), 2), dtype=np.float32)

            for i, match in enumerate(matches):
                points1[i, :] = keypoints1[match.queryIdx].pt
                points2[i, :] = keypoints2[match.trainIdx].pt

            # Find homography with RANSAC method (finds a perspective transformation between two planes)
            h, _ = cv2.findHomography(points2, points1, cv2.RANSAC)
            if h is not None:
                # Compute difference between images and the mean of pixels
                img2New = cv2.warpPerspective(img2, np.asarray(
                    h), (self.RAW_WIDTH, self.RAW_HEIGHT))
                subResult = cv2.subtract(img1, img2New)
                tempVal = cv2.mean(subResult)
                meanResult = tempVal[0]
                # Keep the homography matrix with the most matches and minimun difference
                if meanResult < self.MIN_DIFF:
                    self.MAX_MATCHES = numGoodMatches
                    self.MIN_DIFF = meanResult
                    self.BEST_HOMOGRAPHY = h
                # Dispaly difference between multispectral image and kinect image
                cv2.putText(subResult, "Difference: " + str(meanResult),
                            (10, 1000), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.namedWindow(
                    "Difference between multispectral image and kinect image", cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    "Difference between multispectral image and kinect image", 600, 600)
                cv2.imshow(
                    "Difference between multispectral image and kinect image", subResult)
                # Return an acceptable only homography matrix
                if meanResult < 20:
                    return h
                else:
                    return None
            else:
                # Clear differences display
                subResult = np.zeros(
                    (self.RAW_HEIGHT, self.RAW_WIDTH), np.uint8)
                subResult[:] = 255
                cv2.namedWindow(
                    "Difference between multispectral image and kinect image", cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    "Difference between multispectral image and kinect image", 600, 600)
                cv2.imshow(
                    "Difference between multispectral image and kinect image", subResult)
                return None
        else:
            # Draw top matches 2/2
            cv2.putText(imMatches, "Matches between the images: " + str(numGoodMatches) + " (not enough matches)",
                        (10, 900), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(imMatches, "Matches with min difference: " + str(self.MAX_MATCHES),
                        (10, 950), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(imMatches, "Min captured difference: " + str(self.MIN_DIFF),
                        (10, 1000), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.namedWindow("Matches", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Matches", 600, 600)
            cv2.imshow("Matches", imMatches)
            # Clear differences display
            subResult = np.zeros(
                (self.RAW_HEIGHT, self.RAW_WIDTH), np.uint8)
            subResult[:] = 255
            cv2.namedWindow(
                "Difference between multispectral image and kinect image", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                "Difference between multispectral image and kinect image", 600, 600)
            cv2.imshow(
                "Difference between multispectral image and kinect image", subResult)
            return None

    # Print and save matrix
    def saveHomography(self):
        if(self.BEST_HOMOGRAPHY is not None):
            print(
                "-------------Homography matrix saved to homography1.yaml-------------")
            print(self.BEST_HOMOGRAPHY)
            fs = cv2.FileStorage(self.path, cv2.FILE_STORAGE_WRITE)
            fs.write("homographyMatrix", self.BEST_HOMOGRAPHY)
            fs.release
            print("----------------------End---------------------------\n")
        else:
            print("No homography matrix is available.")

    # Pulish multispectral (mono, rgb8) and depth, ready for 3d reconstruction
    def publishImages(self, img1, img2, img3):
        try:
            # Transform images from OpenCV format to ROS
            self.M_CAMERA_INFO.header.stamp = rospy.Time.from_sec(time.time())
            # Header for multispectral image message
            # Color
            pim_c = self.bridge.cv2_to_imgmsg(img3, self.IME_C)
            pim_c.header = self.M_CAMERA_INFO.header
            # Mono
            pim_m = self.bridge.cv2_to_imgmsg(img1, self.IME_B)
            pim_m.header = self.M_CAMERA_INFO.header
            # Depth
            pim_d = self.bridge.cv2_to_imgmsg(img2, encoding="passthrough")
            pim_d.header = self.M_CAMERA_INFO.header
            # Publish image topics and camera info topic
            self.pub_1_.publish(pim_c)
            self.pub_2_.publish(pim_m)
            self.pub_3_.publish(pim_d)
            self.pub_4_.publish(self.M_CAMERA_INFO)
        except CvBridgeError as e:
            print(e)

    # Read camera parameters and create parameters for camera_info
    def readCameraParameters(self):
        # Multispectral camera parameters
        self.M_CAMERA_INFO = CameraInfo()
        self.M_CAMERA_INFO.header.frame_id = self.FRAME_ID
        self.M_CAMERA_INFO.height = self.RAW_HEIGHT
        self.M_CAMERA_INFO.width = self.RAW_WIDTH
        self.M_CAMERA_INFO.distortion_model = self.D_MODEL
        self.M_CAMERA_INFO.header.seq = 0

        try:
            camera_matrix = rospy.get_param("/camera_matrix/data")
            distortion_coefficients = rospy.get_param(
                "/distortion_coefficients/data")
            rectification_matrix = rospy.get_param(
                "/rectification_matrix/data")
            projection_matrix = rospy.get_param("/projection_matrix/data")

            for i in range(9):
                self.M_CAMERA_INFO.K[i] = camera_matrix[i]
                self.M_CAMERA_INFO.R[i] = rectification_matrix[i]
            for i in range(12):
                self.M_CAMERA_INFO.P[i] = projection_matrix[i]
            for i in range(5):
                self.M_CAMERA_INFO.D.append(distortion_coefficients[i])
        except KeyError:
            rospy.loginfo("Could not retrieve camera parameters.")

        # Read homography matrix parameters
        fs = cv2.FileStorage(self.path, cv2.FILE_STORAGE_READ)
        data = fs.getNode("homographyMatrix").mat()
        fs.release()
        if data is not None:
            homography_matrix = data.reshape(3, 3)
            self.HOMOGRAPHY_MATRIX = homography_matrix
        else:
            rospy.loginfo(
                "Could not retrieve homography matrix. It will cause problem to the publishing mode.")

    # Shut down everything
    def stop(self):
        self.sub_1_.unregister()
        self.sub_2_.unregister()
        self.sub_3_.unregister()
        self.pub_1_.unregister()
        self.pub_2_.unregister()
        self.pub_3_.unregister()
        self.pub_4_.unregister()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Exit")
        rospy.loginfo("Feature matcher node shutted down")


def main(args):
    rospy.init_node("features_registrator", anonymous=True)
    featuresRegistrator = FeaturesRegistrator(args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    featuresRegistrator.stop()


if __name__ == "__main__":
    main(sys.argv)

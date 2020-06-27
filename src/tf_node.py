#!/usr/bin/env python

import turtlesim.srv
import geometry_msgs.msg
import tf
import math
import rospy

if __name__ == "__main__":
    rospy.init_node("tf_node")
    rospy.loginfo("TF node started")
    # Init tf broadcaster and tf listener
    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    listener.waitForTransform(
        "/kinect2_link", "/kinect2_rgb_optical_frame", rospy.Time(), rospy.Duration(4.0))
    # Start loop
    while not rospy.is_shutdown():
        # Listen kinect tf
        try:
            listener.waitForTransform(
                "/kinect2_link", "/kinect2_rgb_optical_frame", rospy.Time(0), rospy.Duration(4.0))
            (tr, rt) = listener.lookupTransform(
                "/kinect2_link", "/kinect2_rgb_optical_frame", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # Broadcast renamed tf
        broadcaster.sendTransform(tr, rt, rospy.Time.now(
        ), "/kinect2_link", "/multispectral_frame")
    # Shut down
    rospy.loginfo("TF node shutted down")

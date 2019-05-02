#!/usr/bin/env python  
import roslib
import rospy

import tf
import turtlesim.msg
import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_republisher')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    # pub = rospy.Publisher('tf_base_xtion', std_msgs.msg.String, queue_size=10)

    listener.waitForTransform("/base_footprint","/xtion_rgb_optical_frame",rospy.Time(),rospy.Duration(5.0)) 
    rospy.sleep(5.0)

    while not rospy.is_shutdown():
        # try:
        # (trans, rot) = listener.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame", rospy.Time(0))
        # roll = np.deg2rad(tf.transformations.euler_from_quaternion(rot)[0]*180/3.141592 - 3.770)
        # pitch = np.deg2rad(tf.transformations.euler_from_quaternion(rot)[1]*180/3.141592)
        # yaw = np.deg2rad(tf.transformations.euler_from_quaternion(rot)[2]*180/3.141592)

        # new_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # br = tf.TransformBroadcaster()
        # br.sendTransform((trans[0], trans[1], trans[2]),
        #                  new_rot,
        #                  rospy.Time.now(),
        #                  "base_footprint",
        #                  "xtion_rgb_optical_frame_angle_correction")
        # (trans, rot) = listener.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame", rospy.Time(0))
        roll = np.deg2rad(-6)
        pitch = 0
        yaw = 0

        new_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         new_rot,
                         rospy.Time.now(),
                         "xtion_rgb_optical_frame_angle_correction",
                         "xtion_rgb_optical_frame")
        # except (tf.LookupException, tf.ConnectivityException):
        #     continue

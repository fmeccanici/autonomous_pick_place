#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import pickle
from joblib import dump, load
import rospy
from aruco_msgs.msg import MarkerArray
import tf
import sys
import re

filename = '/home/fmeccanici/moveit_ws/src/robot_models/marco/autonomous_pick_place/scripts/linear_regressor_angle.joblib'
linear_regressor = load(filename)

pitch = 0
yaw = 0


def callback(data, args):
    a = args[0]
    b = args[1]

    (trans, rot) = listener.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame_roll_correction", rospy.Time(0))

    roll = np.deg2rad(-6)
    # roll = np.deg2rad(-4.640)

    # quaternion = (data.markers[0].pose.pose.orientation.x, data.markers[0].pose.pose.orientation.y, data.markers[0].pose.pose.orientation.z, data.markers[0].pose.pose.orientation.w)
    quaternion = (rot[0], rot[1], rot[2], rot[3])
    orient_euler = tf.transformations.euler_from_quaternion(quaternion)

    
    #z_lin = linear_regressor.predict([[np.deg2rad(90) + orient_euler[0]]])

    # y = a*x + b
    
    z_lin = a*(np.deg2rad(90) + orient_euler[0]) + b
    z_difference = z_lin - (a*np.deg2rad(90+-85)+ b)

    
    #rospy.loginfo("I heard %s",z_difference)
    for i in range(len(data.markers)):
        data.markers[i].pose.pose.position.z -= z_difference

    pub.publish(data)

if __name__ == '__main__':

    rospy.init_node('node_name')
    corrections = [float(number) for number in re.split(',', rospy.get_param("/arg1"))]

    # rospy.loginfo("PARAM NAMES: " + re.split(',', rospy.get_param("/arg1"))[1])

    pub = rospy.Publisher('/marker_detector/markers_calibrated', MarkerArray, queue_size=10)

    listener = tf.TransformListener()
    listener.waitForTransform("/base_footprint","/xtion_rgb_optical_frame_roll_correction",rospy.Time(),rospy.Duration(5.0)) 
    rospy.sleep(5.0)
    rospy.Subscriber("/marker_detector/markers", MarkerArray, callback, corrections)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


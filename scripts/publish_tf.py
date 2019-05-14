import roslib
import rospy
import tf
import sys, traceback
import std_msgs
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('tf_P3AT')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    pub = rospy.Publisher('tf_base_xtion', std_msgs.msg.String, queue_size=10)

    listener.waitForTransform("/base_footprint","/xtion_rgb_optical_frame",rospy.Time(),rospy.Duration(5.0)) 
    rospy.sleep(5.0)
    while not rospy.is_shutdown():

	    try:
	    	# sleep(1)
			(trans, rot) = listener.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame", rospy.Time(0))
			# rospy.loginfo(str(tf.transformations.euler_from_quaternion(rot)[0]*180/3.141592))
			pub.publish(str([tf.transformations.euler_from_quaternion(rot)[0]*180/3.141592, tf.transformations.euler_from_quaternion(rot)[1]*180/3.141592, tf.transformations.euler_from_quaternion(rot)[2]*180/3.141592]))





	    except (tf.LookupException, tf.ConnectivityException):
	        continue


	    

	    # print 'translation: ',trans
	    # print 'rotation: ', rot

	    rate.sleep()
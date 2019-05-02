#!/usr/bin/env python

from os.path import dirname, join

# third party
import rospy
from rospy import ROSInterruptException, Publisher, init_node, get_param, loginfo, Rate
# ROS messages
from std_msgs.msg import Header
from sensor_msgs.msg import Image as ImageMessage
from sensor_msgs.msg import CameraInfo as CameraInfoMessage

import numpy
from imageio import imread

class MockCamera(object):
    
    def __init__(self):
        self._cameraInfoPublisher = Publisher("/xtion/rgb/camera_info_improved", CameraInfoMessage, queue_size=10)
        
        init_node("mock_camera", anonymous=True)


    def run(self):
        rate = Rate(1)
        while not rospy.is_shutdown():

            cameraMessage = self._createCameraInfoMessage()
            self._cameraInfoPublisher.publish(cameraMessage)

            rate.sleep()
    
    def _createCameraInfoMessage(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "xtion_rgb_optical_frame"

        message = CameraInfoMessage()
        message.header = header
        
        message.height = 640
        message.width = 480
        message.distortion_model = "plumb_bob"
        
        
        # message.D = [0.01135616746000704, -0.0837425949431849, -0.003641203664053122, 0.002735692509361075, 0.0]
        # message.K = [524.3530291768766, 0.0, 327.7532296877647, 0.0, 523.6295167359308, 228.0511491808866, 0.0, 0.0, 1.0]
        # message.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # message.P = [519.40966796875, 0.0, 329.275793651701, 0.0, 0.0, 521.64208984375, 226.187870027934, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # message.D = [-0.02817191766837119, -0.04456807713959347, 0.001406068512987967, 0.0004264518716543618, 0]
        # message.K = [511.9643945409745, 0, 314.6780421043781, 0, 512.8531516026462, 253.7017236810161, 0, 0, 1]
        # message.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        # message.P = [502.284423828125, 0, 314.6593117652083, 0, 0, 508.2977905273438, 254.495007154881, 0, 0, 0, 1, 0]

        # message.D = [-0.005619, -0.077768, -0.003193, 0.008015, 0.000000]
        # message.K = [502.809179, 0.000000, 313.329244, 0.000000, 500.768056, 235.729092, 0.000000, 0.000000, 1.000000]

        # message.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        # message.P = [494.846283, 0.000000, 318.289662, 0.000000, 0.000000, 498.635223, 234.452651, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

        message.D = [-0.009740021407495024, -0.06810923458853556, -0.0032394490021143262, 0.0032409844218177944, 0.0]
        message.K = [522.6068524353595, 0.0, 322.48982744054325, 0.0, 524.4301716157004, 232.50832420397631, 0.0, 0.0, 1.0]
        message.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.P = [515.2507934570312, 0.0, 324.70135989618575, 0.0, 0.0, 521.61376953125, 231.24797544667672, 0.0, 0.0, 0.0, 1.0, 0.0]

        # optional parameters, binning and roi, are not set 
        return message
  

if __name__ == "__main__":
    try:    
        node = MockCamera()
        node.run()
    except ROSInterruptException:
        pass

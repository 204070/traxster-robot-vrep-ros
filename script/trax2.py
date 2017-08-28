#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

kinect_depth = Image()
pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)

def transform_callback(data):
    global kinect_depth
    kinect_depth.data = data.data
    print(type(data.data))
    pub.publish(data)


def listener():
    rospy.init_node('traxster_pub', anonymous=True)
    rospy.Subscriber('/vrep/camera/rgb/image_raw', Image, transform_callback)
    # Initial movement.
    pub.publish(kinect_depth)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pasS
#!/usr/bin/env python

import rospy
import vrep
import time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as I 
import array
import numpy


def scan_callback(msg):
    
    print((msg.encoding))

def oyarangeahead():
    rospy.init_node("trax_camera", anonymous=True)
    scan_sub = rospy.Subscriber('/vrep/camera/depth/image_raw', Image, scan_callback)
    rospy.spin()


if __name__ == "__main__":

    #Connection
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.1',19998,True,True,5000,5)

    if clientID != -1:
        print("Connected to remote API Server")

        rospy.init_node('traxster_publisher', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        kinect_rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        kinect_depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)

        errorCodeKinectRGB,kinectRGB=vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_oneshot_wait)
        errorCodeKinectDepth,kinectDepth=vrep.simxGetObjectHandle(clientID,'kinect_rgb',vrep.simx_opmode_oneshot_wait)

        errorHere,resolution,image=vrep.simxGetVisionSensorImage(clientID,kinectRGB,0,vrep.simx_opmode_streaming)
        errorHere,resol,depth=vrep.simxGetVisionSensorDepthBuffer(clientID,kinectDepth,vrep.simx_opmode_streaming)
        time.sleep(1)

        errorHere,resolution,image=vrep.simxGetVisionSensorImage(clientID,kinectRGB,0,vrep.simx_opmode_buffer)
        errorHere,resol,depth=vrep.simxGetVisionSensorDepthBuffer(clientID,kinectDepth,vrep.simx_opmode_buffer)

        while not rospy.is_shutdown():
            errorHere,resolution,image=vrep.simxGetVisionSensorImage(clientID,kinectRGB,0,vrep.simx_opmode_buffer)
            image_byte_array = array.array('b', image)
            image_buffer = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
            img2 = numpy.asarray(image_buffer)
            msg_kinect_rgb = CvBridge().cv2_to_imgmsg(img2, "bgr8")
            kinect_rgb_pub.publish(msg_kinect_rgb)
            rate.sleep()



            
    else:
            print "Connection not succesfull"
            sys.exit("Could not connect")

    #oyarangeahead()
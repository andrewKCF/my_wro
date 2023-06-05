#!/usr/bin/env python3
#!coding=utf-8

#right code !
#write by leo at 2018.04.26
#function: 
#display the frame from another node.

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("webcam" , cv_img)
        cv2.waitKey(3)
    else:
        pass

def webcam_display():
    rospy.init_node('webcam_display', anonymous=True)

    # make a video_object and init the video object
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    webcam_display()

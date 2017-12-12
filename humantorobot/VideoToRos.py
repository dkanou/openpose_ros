#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

rospy.init_node('VideoPublisher', anonymous=True)
bridge = CvBridge()
rate = rospy.Rate(30)
VideoRaw = rospy.Publisher('VideoRaw', Image, queue_size=100)
t = 0 
while t < 20:
    time.sleep(1)
    print("Sleeping..." + str(t))
    t+=1

cam = cv2.VideoCapture('brianposes.avi')

while cam.isOpened():
    meta, frame = cam.read()

    #Original Image
    print(type(frame))  
    print(frame.shape)
    frame = np.uint8(frame) 
    msg_frame = bridge.cv2_to_imgmsg(frame, "bgr8")
    print(type(msg_frame))
    VideoRaw.publish(msg_frame)
    #cam.release()
    #time.sleep(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    rate.sleep()


#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def capture(msg):
    global i
    global bridge
    i += 1

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    cv2.imshow("KIR", frame)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite('./saves/{}.JPG'.format(i), frame)


rospy.init_node('pose_estimator', anonymous=True)
rospy.Subscriber("/camera/rgb/image_color", Image, capture, )
# 10hz
i = 0
bridge = CvBridge()
rate = rospy.Rate(1)
rospy.spin()

#!/usr/bin/env python
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def find_circle(msg):
    global bridge

    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img, 13)
    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 50, param1=100, param2=30, minRadius=5, maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :2]:
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 10)
            cv2.circle(cimg, (i[0], i[1]), 10, (0, 0, 255), 10)

    img_pub.publish(bridge.cv2_to_imgmsg(cimg, "bgr8"))

    # cv2.imshow('detected circles', cimg)
    # if cv2.waitKey(1) & 0xff == ord('q'):
    #     return



if __name__ == "__main__":
    bridge = CvBridge()
    img_pub = rospy.Publisher('/localizer_img', Image, queue_size=1)
    rospy.init_node('localizer_circle', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, find_circle)
    rate = rospy.Rate(10)

    rospy.spin()


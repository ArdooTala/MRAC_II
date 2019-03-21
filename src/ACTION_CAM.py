#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2


if __name__ == '__main__':
    rospy.init_node('action_cam')
    img_pub = rospy.Publisher('KINNECT', Image, queue_size=1)


    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        # cv2.imshow("KIR", frame)
        # if cv2.waitKey(1) & 0xff == ord('q'):
        #     break

        img_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

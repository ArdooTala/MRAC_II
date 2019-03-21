#!/usr/bin/env python
# license removed for brevity

import numpy as np
import cv2, PIL, os
from cv2 import aruco
import pickle
import pandas as pd

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf import transformations


def to_pose_stamped(tvec, rvec):
    x, y, z = tvec

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = ""
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    M1 = cv2.Rodrigues(rvec)[0]
    rospy.loginfo(cv2.Rodrigues(rvec)[0])
    # quaternion = transformations.quaternion_from_matrix(cv2.Rodrigues(rvec)[0])
    # quaternion = transformations.quaternion_from_euler(*rvec)
    r = np.math.sqrt(float(1) + M1[0, 0] + M1[1, 1] + M1[2, 2]) * 0.5
    i = (M1[2, 1] - M1[1, 2]) / (4 * r)
    j = (M1[0, 2] - M1[2, 0]) / (4 * r)
    k = (M1[1, 0] - M1[0, 1]) / (4 * r)

    pose.pose.orientation.x = i
    pose.pose.orientation.y = j
    pose.pose.orientation.z = k
    pose.pose.orientation.w = r

    return pose


def find_ar(msg):
    global parameters
    global criteria
    global size_of_marker
    global length_of_axis
    global bridge
    global aruco_dict

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners:
        # SUB PIXEL DETECTION
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, size_of_marker, mtx, dist)
        print _
        frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        for i in range(len(tvecs)):
            gray = aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

        print "\n", "_"*4, "TRANSFORM", "_"*40, "\n"
        print "T: ", tvecs
        print "R: ", rvecs
        crd_pub.publish(to_pose_stamped(tvecs[0][0], rvecs[0][0]))
    img_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    # cv2.imshow("KUN", frame)
    # cv2.waitKey(1)


if __name__ == "__main__":
    with open("/home/ardoo/catkin_ws/src/my_pcl_tutorial/src/Calibration_Parameters.pickle", 'rb') as pkl:
        ret, mtx, dist = pickle.load(pkl)

    print "Calibration Parameters Loaded . . ."
    print "\nMTX:\n", mtx
    print "\nDIST:\n", dist

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    size_of_marker = 0.114  # side lenght of the marker in meter
    length_of_axis = 0.5

    bridge = CvBridge()

    img_pub = rospy.Publisher('/marker/img', Image, queue_size=1)
    crd_pub = rospy.Publisher('/marker/pose', PoseStamped, queue_size = 1)
    rospy.init_node('localizer_marker', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, find_ar)
    rate = rospy.Rate(10)  # 10hz

    rospy.spin()

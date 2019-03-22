#!/usr/bin/env python
# license removed for brevity

import numpy as np
import cv2, PIL, os
from cv2 import aruco
import pickle
import pandas as pd
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf import transformations

from AR2Transform import Detector
from Transforms import Transformations


def to_pose_stamped(t_matrix):

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = ""
    pose.pose.position.x = t_matrix[0, 3]
    pose.pose.position.y = t_matrix[1, 3]
    pose.pose.position.z = t_matrix[2, 3]

    quaternion = transformations.quaternion_from_matrix(t_matrix)
    d = math.sqrt(quaternion[0]**2+quaternion[1]**2+quaternion[2]**2+quaternion[3]**2)

    pose.pose.orientation.x = quaternion[0]/d
    pose.pose.orientation.y = quaternion[1]/d
    pose.pose.orientation.z = quaternion[2]/d
    pose.pose.orientation.w = quaternion[3]/d

    return pose



def find_ar(msg):
    global detector, tr

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    imaxis, translations = detector.find_ar(frame)
    if translations:
        if len(translations) == 3:
            trans = [o[1] for o in sorted(translations.items(), key=lambda x:x[0])]

            transformations_matrix = tr.calculate_transformation(*trans)

            crd_pub.publish(to_pose_stamped(transformations_matrix))
            img_pub.publish(bridge.cv2_to_imgmsg(imaxis, "bgr8"))

    """
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
        # cv2.imshow("KUN", frame)
    # cv2.waitKey(1)
"""

if __name__ == "__main__":
    detector = Detector(0.08,
                        "/home/ardoo/catkin_ws/src/my_pcl_tutorial/src/Calibration_Parameters.pickle",
                        [0, 8, 9])
    tr = Transformations([2500, 1000, 600], [2500, -1000, 600], [2900, 1000, 600])
    bridge = CvBridge()

    img_pub = rospy.Publisher('/marker/img', Image, queue_size=1)
    crd_pub = rospy.Publisher('/marker/pose', PoseStamped, queue_size=1)
    rospy.init_node('localizer_marker', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_color", Image, find_ar)

    rospy.spin()
    """

    print "Calibration Parameters Loaded . . ."
    print "\nMTX:\n", mtx
    print "\nDIST:\n", dist

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    size_of_marker = 0.114  # side lenght of the marker in meter
    length_of_axis = 0.5

    

    rospy.Subscriber("/camera/rgb/image_color", Image, find_ar)
    rate = rospy.Rate(10)  # 10hz

    
"""
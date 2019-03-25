#!/usr/bin/env python
# license removed for brevity

import numpy as np
import cv2, PIL, os
from cv2 import aruco
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

            crd1_pub.publish(to_pose_stamped(transformations_matrix))
            crd2_pub.publish(to_pose_stamped(tr.robot2marker_transform))

    img_pub.publish(bridge.cv2_to_imgmsg(imaxis, "bgr8"))


if __name__ == "__main__":
    detector = Detector(0.055,
                        "/home/ardoo/catkin_ws/src/my_pcl_tutorial/src/Calibration_Parameters.pickle",
                        [0, 3, 4])
    tr = Transformations([0.328, 0.347, 0.017], [.458, -0.309, 0.016], [0.491, 0.353, .016])
    bridge = CvBridge()

    img_pub = rospy.Publisher('/marker/img', Image, queue_size=1)
    crd1_pub = rospy.Publisher('/marker/pose', PoseStamped, queue_size=1)
    crd2_pub = rospy.Publisher('/robot/pose', PoseStamped, queue_size=1)

    rospy.init_node('localizer_marker', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_color", Image, find_ar)

    rospy.spin()

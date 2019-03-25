#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
from my_pcl_tutorial.srv import lookup_transform, lookup_transformResponse


def publish_transform(req):
    resp = geometry_msgs.msg.Transform()

    global trans, rot
    if not trans or not rot:
        resp.translation.x = 0
        resp.translation.y = 0
        resp.translation.z = 0
        resp.rotation.x = 0
        resp.rotation.y = 0
        resp.rotation.z = 0
        resp.rotation.w = 1
    else:
        (resp.translation.x,
        resp.translation.y,
        resp.translation.z) = trans
        (resp.rotation.x,
        resp.rotation.y,
        resp.rotation.z,
        resp.rotation.w) = rot

    return lookup_transformResponse(resp)


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    s = rospy.Service('robot2kinect_transform', lookup_transform, publish_transform)

    listener = tf.TransformListener()

    # robot2kinect = rospy.Publisher('robot2kinect', geometry_msgs.msg.Twist,queue_size=1)
    trans = None
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/ABB', '/camera_rgb_optical_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print trans
        # robot2kinect.publish(cmd)

        rate.sleep()
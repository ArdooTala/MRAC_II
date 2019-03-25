#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


def handle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z),
                     (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     "ABB",
                     "parker")


if __name__ == '__main__':
    rospy.init_node('robotizer_tf')
    # rr = tf.StaticTransformBroadcaster()
    rospy.Subscriber('/robot/pose',
                     PoseStamped,
                     handle_pose
                     )

    rospy.spin()

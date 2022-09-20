#!/usr/bin/env python

import rospy
import roslib
import tf
import math
import time
import quaternion
from geometry_msgs.msg import Point, Quaternion, PoseStamped

if __name__ == '__main__':
    rospy.init_node('body_pose_publisher')
    listener = tf.TransformListener()
    pub = rospy.Publisher("body_pose", PoseStamped, queue_size=1)
    rate = rospy.Rate(50.0)
    t = [0]*5
    for i in range(5):
        t[i] = rospy.Time.now()
    time.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            for i in range(4):
                t[i] = t[i+1]
            t[4] = rospy.Time.now()
            listener.waitForTransform("/odom_ground", "/BODY", t[0], rospy.Duration(3.0))
            (trans,rot) = listener.lookupTransform('/odom_ground', '/BODY', t[0])
            msg = PoseStamped()
            msg.header.stamp = t[0]
            msg.pose.position.x = trans[0]
            msg.pose.position.y = trans[1]
            msg.pose.position.z = trans[2]
            msg.pose.orientation.x = rot[0]
            msg.pose.orientation.y = rot[1]
            msg.pose.orientation.z = rot[2]
            msg.pose.orientation.w = rot[3]
            pub.publish(msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("error")
            continue

        rate.sleep()

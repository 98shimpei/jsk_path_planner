#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

pointcloud_msg = None

def callback(msg):
    global pointcloud_msg
    pointcloud_msg = msg

rospy.init_node("pointcloud_pub_regulator")
sub = rospy.Subscriber("~input", PointCloud2, callback)
pub = rospy.Publisher("~output", PointCloud2, queue_size=3)
rate = rospy.get_param("~rate", 1)
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    if pointcloud_msg is not None:
        pub.publish(pointcloud_msg)
    else:
        print("input topic has not been published yet")
    r.sleep()

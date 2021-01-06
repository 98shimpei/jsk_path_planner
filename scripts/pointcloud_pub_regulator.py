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
delay_offset_ms = rospy.get_param("~delay_offset_ms", 0)
# offset_ms = 78  ## 2021/1/6 omori desktop
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    if pointcloud_msg is not None:
        stamp = pointcloud_msg.header.stamp
        stamp.nsecs += (delay_offset_ms * 1000000)
        if stamp.nsecs > 1000000000:
            stamp.nsecs %= 1000000000
            stamp.secs += 1

        pub.publish(pointcloud_msg)
        pointcloud_msg = None
    else:
        print("input topic has not been published yet")
    r.sleep()

#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PolygonArray
from safe_footstep_planner.msg import SteppableRegion

def callback(msg):
    output_msg = PolygonArray()
    output_msg.header     = msg.header
    output_msg.polygons   = msg.polygons
    output_msg.labels     = msg.labels
    output_msg.likelihood = msg.likelihood

    # fill empty
    output_msg.header.stamp = rospy.Time(0)
    for i in range(len(output_msg.polygons)):
        output_msg.polygons[i].header = output_msg.header

    pub.publish(output_msg)

rospy.init_node("steppable_region_visualizer", anonymous=True)
sub = rospy.Subscriber("~input", SteppableRegion, callback)
pub = rospy.Publisher("~output", PolygonArray, queue_size=3)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")

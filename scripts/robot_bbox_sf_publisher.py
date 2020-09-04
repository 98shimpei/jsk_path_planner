#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox
from geometry_msgs.msg import Point, Quaternion, Vector3

rospy.init_node("robot_bbox_sf_publisher", anonymous=True)
p = rospy.Publisher("/robot_bbox_sf", BoundingBox, queue_size=10)
r = rospy.Rate(100)

bbox_msg = BoundingBox()
# bbox_msg.header.frame_id = 'body_on_odom'
bbox_msg.header.frame_id = 'BODY'

position = Point(0.0, 0.0, 0.0)
orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
dimensions = Vector3(0.8, 1.6, 1.6)

bbox_msg.pose.position = position
bbox_msg.pose.orientation = orientation
bbox_msg.dimensions = dimensions

while not rospy.is_shutdown():
  p.publish(bbox_msg)
  r.sleep()

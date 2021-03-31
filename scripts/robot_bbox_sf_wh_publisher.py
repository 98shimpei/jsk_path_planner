#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Point, Quaternion, Vector3

rospy.init_node("robot_bbox_sf_wh_publisher", anonymous=True)
p = rospy.Publisher("/robot_bbox_sf_wh", BoundingBoxArray, queue_size=10)
r = rospy.Rate(100)

# BODY filter
body_bbox_msg = BoundingBox()
body_bbox_msg.header.frame_id = 'BODY'
body_bbox_msg.pose.position = Point(0.0, 0.0, 0.0)
body_bbox_msg.pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
body_bbox_msg.dimensions = Vector3(0.8, 1.6, 1.6)

# Right hand filter
rh_bbox_msg = BoundingBox()
rh_bbox_msg.header.frame_id = 'R_thk_palm'
rh_bbox_msg.pose.position = Point(0.15, 0.0, 0.0)
rh_bbox_msg.pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
rh_bbox_msg.dimensions = Vector3(0.3, 0.3, 0.15)

# Left hand filter
lh_bbox_msg = BoundingBox()
lh_bbox_msg.header.frame_id = 'L_thk_palm'
lh_bbox_msg.pose.position = Point(0.15, 0.0, 0.0)
lh_bbox_msg.pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
lh_bbox_msg.dimensions = Vector3(0.3, 0.3, 0.15)

# make bbox array
bbox_array_msg = BoundingBoxArray()
bbox_array_msg.boxes = [body_bbox_msg, rh_bbox_msg, lh_bbox_msg]

while not rospy.is_shutdown():
  p.publish(bbox_array_msg)
  r.sleep()

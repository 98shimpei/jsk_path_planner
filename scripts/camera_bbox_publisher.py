#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty as EmptySrv
from jsk_recognition_msgs.msg import BoundingBox
from jsk_footstep_msgs.msg import FootstepArray
from geometry_msgs.msg import Point, Quaternion, Vector3

rospy.init_node("camera_bbox_publisher", anonymous=True)
p = rospy.Publisher("/d435_bbox", BoundingBox, queue_size=10)
r = rospy.Rate(100)

bbox_msg = BoundingBox()
bbox_msg.header.frame_id = '/chest_camera_optical_frame'

position = Point(0.0, 0.0, 0.0)
orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
dimensions = Vector3(0.09, 0.025, 0.025)

bbox_msg.pose.position = position
bbox_msg.pose.orientation = orientation
bbox_msg.dimensions = dimensions

while not rospy.is_shutdown():
  p.publish(bbox_msg)
  r.sleep()

#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty as EmptySrv
from jsk_recognition_msgs.msg import BoundingBox
from jsk_footstep_msgs.msg import FootstepArray
from geometry_msgs.msg import Point, Quaternion, Vector3

rospy.init_node("object_bbox_publisher", anonymous=True)
p = rospy.Publisher("/object_bbox", BoundingBox, queue_size=10)
r = rospy.Rate(100)
transport_object = rospy.get_param('~transport_object', "wheelbarrow")
set_marker_id = rospy.get_param('~set_marker_id', False)
marker_id = rospy.get_param('~marker_id', "")

# rospy.loginfo("Clearing /accumulated_heightmap/reset due to change of robot_bbox")
# srv = rospy.ServiceProxy("/accumulated_heightmap/reset", EmptySrv)
# try:
#   srv()
# except:
#   rospy.logerr("Failed to reset /accumulated_heightmap")

if transport_object == "push_cart":
  frame_id = "ar_marker_52"
  position = Point(0.75, 0.0, 1.25)
  orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
  dimensions = Vector3(1.5, 1.2, 3.0)
elif transport_object == "wheelbarrow":
  frame_id = "ar_marker_52"
  # position = Point(-0.45, 0.0, 0.2)
  # orientation = Quaternion(0, 0.3420201, 0, 0.9396926)
  # dimensions = Vector3(1.2, 0.7, 0.8)
  position = Point(-0.3, 0.0, 0.3)
  orientation = Quaternion(0, 0, 0, 1)
  dimensions = Vector3(0.9, 0.7, 1.1)
else:
  frame_id = "ar_marker_52"
  position = Point(0.0, 0.0, 1.25)
  orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
  dimensions = Vector3(0.5, 1.0, 3.0)

if set_marker_id:
  frame_id = "ar_marker_" + str(marker_id)

bbox_msg = BoundingBox()
bbox_msg.header.frame_id = frame_id
bbox_msg.pose.position = position
bbox_msg.pose.orientation = orientation
bbox_msg.dimensions = dimensions

while not rospy.is_shutdown():
  p.publish(bbox_msg)
  r.sleep()

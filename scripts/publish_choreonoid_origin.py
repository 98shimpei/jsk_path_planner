#!/usr/bin/env python

import rospy
import tf
import tf.msg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

def callback(msg):
    t = TransformStamped()
    t.header = msg.header
    t.child_frame_id = "base_footprint"

    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    tfm = tf.msg.tfMessage([t])
    pub_tf.publish(tfm)

    print("publish choreonoid origin")


if __name__ == '__main__':
    rospy.init_node('choreonoid_origin_publisher', anonymous=True)
    pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)
    pose_sub = rospy.Subscriber("/PR2/ground_truth_pose", PoseStamped, callback)
    rospy.spin()

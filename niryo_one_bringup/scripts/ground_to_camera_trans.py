#!/usr/bin/env python

from pprint import pprint
import sys
import yaml
import subprocess
import copy
import rospy
import time
import tf
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def test_camera():
	rospy.init_node("camera_test")

	bcaster = tf2_ros.StaticTransformBroadcaster()

	transform_msg = geometry_msgs.msg.TransformStamped()
	transform_msg.header.frame_id = "ground_link"
	transform_msg.child_frame_id  = "camera_link"








	transform_msg.transform.translation.x = 0.411237
	transform_msg.transform.translation.y = 0.273886
	transform_msg.transform.translation.z = 0.293974

	quat_init = [0.19136092, 0.88362043, -0.40819979, -0.12636799]
	
	q_rot = tf.transformations.quaternion_from_euler(-pi/36, -pi/36, 0)

	q_final = tf.transformations.quaternion_multiply(q_rot, quat_init)

	print q_final	
	
	transform_msg.transform.rotation.x = q_final[0]
	transform_msg.transform.rotation.y = q_final[1]
	transform_msg.transform.rotation.z = q_final[2]
	transform_msg.transform.rotation.w = q_final[3]
	bcaster.sendTransform(transform_msg)
	rospy.spin()
	#time.sleep(5)


if __name__ == "__main__":
	test_camera()





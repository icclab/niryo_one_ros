#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
from pprint import pprint
import sys
import yaml
import subprocess
import copy
import rospy
import time
import tf
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String

def calibrate_camera():
	rospy.init_node('camera_calibration')
	ar_marker_id = "13"
	ar_pose = [0.003, -0.23, 0.0, 0.707, 0.0, 0.0, 0.707]

	listener = tf.TransformListener(cache_time=rospy.Duration(10.0))
	#transformer = tf.TransformerROS(True, rospy.Duration(10.0))

	time.sleep(10)
	t = rospy.Time.now()

	listener.waitForTransform("ground_link", "camera_link", t, rospy.Duration(4.0))
	(trans, quat) = listener.lookupTransform("ground_link", "camera_link", t)

	pose_msg = geometry_msgs.msg.PoseStamped()
	pose_msg.header.frame_id = "ar_marker_" + ar_marker_id
	pose_msg.header.stamp = t
	pose_msg.pose.position.x = trans[0]
	pose_msg.pose.position.y = trans[1]
	pose_msg.pose.position.z = trans[2]
	pose_msg.pose.orientation.x = quat[0]
	pose_msg.pose.orientation.y = quat[1]
	pose_msg.pose.orientation.z = quat[2]
	pose_msg.pose.orientation.w = quat[3]
	listener.waitForTransform("ar_marker_"+ar_marker_id, "ground_link", t, rospy.Duration(4.0))
	pose_final = listener.transformPose("ground_link", pose_msg)

	#trans = (pose_final.pose.position.x, pose_final.pose.position.y, pose_final.pose.position.z)
	#quat  = (pose_final.pose.orientation.x, pose_final.pose.orientation.y, pose_final.pose.orientation.z, pose_final.pose.orientation.w)

	print("Ground_link-to-camera_link transform: [x y z | x y z w]")
	pprint(trans+quat)

	with open('camera_params.yaml', 'w') as outfile:
		yaml.dump(trans+quat, outfile, explicit_start=True, default_flow_style=False)

if __name__ == '__main__':
	calibrate_camera()

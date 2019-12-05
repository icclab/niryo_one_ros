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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def niryo_demo():
	#Approach vector and offset distance to compensate for gripper length
	approach = [0, 0, -1]
	grasp_offset = -0.09

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_niryo_robot')

	n = NiryoOne()
	n.calibrate_auto()
	n.activate_learning_mode(False)
	n.change_tool(TOOL_GRIPPER_3_ID)
   	n.open_gripper(TOOL_GRIPPER_3_ID, 200)
	print("[GRIPPER 3 OPENED]")

	robot = moveit_commander.RobotCommander()
	#scene = moveit_commander.PlanningSceneInterface()

	group_name = "arm"
	group = moveit_commander.MoveGroupCommander(group_name)
	group.set_planning_time(10)

	#Read camera-to-ground transform from yaml file and publish
	with open('camera_params.yaml', 'r') as infile:
		cam_pose = yaml.load(infile)

	camera_static_transform = subprocess.Popen(
				["rosrun", "tf", "static_transform_publisher", str(cam_pose[0]), str(cam_pose[1]), str(cam_pose[2]), 
				str(cam_pose[3]), str(cam_pose[4]), str(cam_pose[5]), str(cam_pose[6]), "camera_depth_optical_frame", "ground_link", "100"])

	print("[INFO] Camera-robot link established")

	#Let the buffer fill so we can get the frame list
	listener = tf.TransformListener()
	time.sleep(2)
	frame_list = listener.getFrameStrings()

	#One object only
	object_frame = [e for e in frame_list if e[:7] == "object_"][0]

	listener.waitForTransform("base_link", object_frame, rospy.Time(), rospy.Duration(2.0))
	(trans, quat) = listener.lookupTransform("base_link", object_frame, rospy.Time(0))

	#print "Translation [x, y, z]    = " + str(trans)
	#print "Orientation [x, y, z, w] = " + str(quat)

	pick_target = geometry_msgs.msg.Pose()
	pick_target.position.x    	= trans[0] + grasp_offset * approach[0]
	pick_target.position.y		= trans[1] + grasp_offset * approach[1]
	pick_target.position.z		= trans[2] + grasp_offset * approach[2]
	#pick_target.orientation.x	= quat[0]
	#pick_target.orientation.y	= quat[1]
	#pick_target.orientation.z	= quat[2]
	#pick_target.orientation.w 	= quat[3]
	
	pick_target.orientation.x	= 0.0
	pick_target.orientation.y	= 1.0/math.sqrt(2)
	pick_target.orientation.z	= 0.0
	pick_target.orientation.w 	= 1.0/math.sqrt(2)
	group.set_pose_target(pick_target)

	pprint(pick_target)

	plan = group.plan()
	group.go(wait=True)

	n.close_gripper(TOOL_GRIPPER_3_ID, 200)
	print("[GRIPPER 3 CLOSED]")

	place_target = geometry_msgs.msg.Pose()
	place_target.position.x =  pick_target.position.x
	place_target.position.y =  pick_target.position.y
	place_target.position.z =  pick_target.position.z + 0.1 #Lift the cube 10cm
	place_target.orientation.x	= 0.0
	place_target.orientation.y	= 1.0/math.sqrt(2)
	place_target.orientation.z	= 0.0
	place_target.orientation.w 	= 1.0/math.sqrt(2)

	group.set_pose_target(place_target)
	plan = group.plan()
	group.go(wait=True)

	#Place cube in the -y position of starting position
	place_target = geometry_msgs.msg.Pose()
	place_target.position.x =  pick_target.position.x
	place_target.position.y = -pick_target.position.y
	place_target.position.z =  pick_target.position.z
	place_target.orientation.x	= 0.0
	place_target.orientation.y	= 1.0/math.sqrt(2)
	place_target.orientation.z	= 0.0
	place_target.orientation.w 	= 1.0/math.sqrt(2)

	group.set_pose_target(place_target)
	plan = group.plan()
	group.go(wait=True)
	
	n.open_gripper(TOOL_GRIPPER_3_ID, 200)
	print("[GRIPPER 3 OPENED]")

	#Move to resting position
	resting_joints = [0, 0.64, -1.39, 0, 0, 0]
	n.move_joints(resting_joints)

	n.close_gripper(TOOL_GRIPPER_3_ID, 200)
	print("[GRIPPER 3 CLOSED]")

	n.activate_learning_mode(True)

	camera_static_transform.kill()

	moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
	try:
		niryo_demo()
	except rospy.ROSInterruptException:
		n.activate_learning_mode(True)

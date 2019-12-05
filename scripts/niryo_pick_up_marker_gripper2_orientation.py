#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
from pprint import pprint
import sys
import subprocess
import copy
import rospy
import time
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion
import visualization_msgs.msg
import math
from pyquaternion import Quaternion
import time
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

marker_subscriber = None;
marker_pose = geometry_msgs.msg.PoseStamped();
#Change this depending on the marker used
marker_id = 1

def parse_visualization_msg(data):
    if (data.id == marker_id):
      marker_pose.pose= data.pose;
      marker_pose.header.frame_id = data.header.frame_id;
      #print(data.pose);
      

def get_object_marker_position():    
    marker_subscriber = rospy.Subscriber("/visualization_marker", visualization_msgs.msg.Marker, parse_visualization_msg)
    print("Subscribing to /visualization_marker");
    while (marker_pose.header.frame_id == ""):
      time.sleep(0.1)
    marker_subscriber.unregister();
    print("Unsubscribing from /visualization_marker");

def niryo_demo():
  #Approach vector and offset distance to compensate for gripper length
  approach = [0, 0, -1]
  grasp_offset = -0.12

  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('moveit_niryo_robot')

  n = NiryoOne()
  n.calibrate_auto()
  n.activate_learning_mode(False)
  n.change_tool(TOOL_GRIPPER_2_ID)
  n.open_gripper(TOOL_GRIPPER_2_ID, 200)
  print("[GRIPPER 3 OPENED]")

  robot = moveit_commander.RobotCommander()
  #scene = moveit_commander.PlanningSceneInterface()

  group_name = "arm"
  group = moveit_commander.MoveGroupCommander(group_name)
  group.set_planning_time(10)

  listener = tf.TransformListener()
  #Let the buffer fill so we can get the frame list
  time.sleep(1)
  frame_list = listener.getFrameStrings()

#	listener.waitForTransform("camera_color_optical_frame", "ground_link", rospy.Time(), rospy.Duration(2.0))
#	(trans, quat) = listener.lookupTransform("camera_color_optical_frame", "ground_link", rospy.Time(0))
#	camera_static_transform = subprocess.Popen(
#				["rosrun", "tf", "static_tranform_publisher", str(trans[0]), str(trans[1]), str(trans[2]), str(quat[0]), str(quat[1]), str(quat[2]), str(quat[3]), "100"])

#	print("[INFO] Camera-robot link established, you can remove the ARUCO marker now.")
#	rospy.sleep(5)
	#One object only
#	marker_frame = [e for e in frame_list if e[:7] == "object_"][0]
  get_object_marker_position()
  print("Transforming point from frame: " + marker_pose.header.frame_id)
  listener.waitForTransform("base_link", marker_pose.header.frame_id, rospy.Time(), rospy.Duration(2.0))
#	(trans, quat) = listener.lookupTransform("base_link", marker_frame, rospy.Time(0))
  marker_point = listener.transformPose("base_link", marker_pose) # the marker position in arm base frame
  pprint(marker_point)

  #print "Translation [x, y, z]    = " + str(trans)
  #print "Orientation [x, y, z, w] = " + str(quat)

  q = Quaternion(marker_point.pose.orientation.w,
                 marker_point.pose.orientation.x,
                 marker_point.pose.orientation.y,
                 marker_point.pose.orientation.z)
  z_new = q.rotate([0.0, 0.0, 1.0])
  theta = math.acos(z_new[0])

  qy = Quaternion(axis = [0, 1, 0], angle = math.pi / 2)
  if z_new[1] > 0:
    sign = -1
  else:
    sign = 1
  qz = Quaternion(axis = [0, 0, 1], angle = sign * (math.pi/2 - theta))
  q_gripper = qz * qy

  point = marker_point.pose.position

  pick_target = geometry_msgs.msg.Pose()
  pick_target.position.x    = point.x + grasp_offset * approach[0]
  pick_target.position.y		= point.y + grasp_offset * approach[1]
  pick_target.position.z		= point.z + grasp_offset * approach[2]
  #pick_target.orientation.x	= 0.0
  #pick_target.orientation.y	= 1.0/math.sqrt(2)
  #pick_target.orientation.z	= 0.0
  #pick_target.orientation.w 	= 1.0/math.sqrt(2)
  pick_target.orientation.x  = q_gripper[1]
  pick_target.orientation.y  = q_gripper[2]
  pick_target.orientation.z  = q_gripper[3]
  pick_target.orientation.w  = q_gripper[0]
  group.set_pose_target(pick_target)

  pprint(pick_target)

  #pprint(pick_target)

  plan = group.plan()
  group.go(wait=True)

  n.close_gripper(TOOL_GRIPPER_2_ID, 200)
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
  place_target.orientation.w  = 1.0/math.sqrt(2)

  group.set_pose_target(place_target)
  plan = group.plan()
  group.go(wait=True)

  n.open_gripper(TOOL_GRIPPER_2_ID, 200)
  print("[GRIPPER 3 OPENED]")

  #Move to resting position
  resting_joints = [0, 0.64, -1.39, 0, 0, 0]
  n.move_joints(resting_joints)

  n.close_gripper(TOOL_GRIPPER_2_ID, 200)
  print("[GRIPPER 3 CLOSED]")

  n.activate_learning_mode(True)

  moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
  try:
	  niryo_demo()
  except rospy.ROSInterruptException:
	  n.activate_learning_mode(True)


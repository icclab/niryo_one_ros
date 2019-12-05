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
import visualization_msgs.msg
import math
import time
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

marker_subscriber = None;
marker_pose = geometry_msgs.msg.PointStamped();
marker_ids = [0, 1]

def parse_visualization_msg(data):
    if (data.id in marker_ids):
      global marker_subscriber
      global marker_pose
      marker_pose.point = data.pose.position;
      marker_pose.header.frame_id = data.header.frame_id;
      print(marker_pose);
      marker_subscriber.unregister();
      print("Unsubscribing from /visualization_marker");

def get_object_marker_position():    
    global marker_subscriber
    marker_subscriber = rospy.Subscriber("/visualization_marker", visualization_msgs.msg.Marker, parse_visualization_msg)
    print("Subscribing to /visualization_marker");
    while (marker_pose.header.frame_id == ""):
      time.sleep(0.1)

def niryo_demo():
  #Approach vector and offset distance to compensate for gripper length
  approach = [0, 0, -1]
  grasp_offset = -0.12

  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('moveit_niryo_robot')

  n = NiryoOne()
  n.calibrate_auto()
  n.change_tool(TOOL_GRIPPER_2_ID)
  global marker_pose;
  robot = None;
  tries = 1;
  while (robot == None and tries < 5):
    try:
      robot = moveit_commander.RobotCommander()
    except:
      print("Could not connect to move_it action server")
  #scene = moveit_commander.PlanningSceneInterface()

  group_name = "arm"
  group = moveit_commander.MoveGroupCommander(group_name)
  group.set_planning_time(10)
  n.activate_learning_mode(False)

  listener = tf.TransformListener()
  #Let the buffer fill so we can get the frame list
  time.sleep(3)
  frame_list = listener.getFrameStrings()

#	listener.waitForTransform("camera_color_optical_frame", "ground_link", rospy.Time(), rospy.Duration(2.0))
#	(trans, quat) = listener.lookupTransform("camera_color_optical_frame", "ground_link", rospy.Time(0))
#	camera_static_transform = subprocess.Popen(
#				["rosrun", "tf", "static_tranform_publisher", str(trans[0]), str(trans[1]), str(trans[2]), str(quat[0]), str(quat[1]), str(quat[2]), str(quat[3]), "100"])

#	print("[INFO] Camera-robot link established, you can remove the ARUCO marker now.")
#	rospy.sleep(5)
	#One object only
#	marker_frame = [e for e in frame_list if e[:7] == "object_"][0]
  while True:
    get_object_marker_position()
    n.open_gripper(TOOL_GRIPPER_2_ID, 200)
    print("[GRIPPER 2 OPENED]")
    print("Transforming point from frame: " + marker_pose.header.frame_id)
    listener.waitForTransform("base_link", marker_pose.header.frame_id, rospy.Time(), rospy.Duration(2.0))
  #	(trans, quat) = listener.lookupTransform("base_link", marker_frame, rospy.Time(0))
    marker_point = listener.transformPoint("base_link", marker_pose) # the marker position in arm base frame
    marker_pose = geometry_msgs.msg.PointStamped();
    #print "Translation [x, y, z]    = " + str(trans)
    #print "Orientation [x, y, z, w] = " + str(quat)

    point = marker_point.point
    pick_target = geometry_msgs.msg.Pose()
    pick_target.position.x    	= point.x + grasp_offset * approach[0]
    pick_target.position.y		= point.y + grasp_offset * approach[1]
    pick_target.position.z		= point.z + grasp_offset * approach[2]
    pick_target.orientation.x	= 0.0
    pick_target.orientation.y	= 1.0/math.sqrt(2)
    pick_target.orientation.z	= 0.0
    pick_target.orientation.w 	= 1.0/math.sqrt(2)
    group.set_pose_target(pick_target)

    pprint(pick_target)

    plan = group.plan()
    group.go(wait=True)

    n.close_gripper(TOOL_GRIPPER_2_ID, 200)
    print("[GRIPPER 2 CLOSED]")

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

    n.open_gripper(TOOL_GRIPPER_2_ID, 200)
    print("[GRIPPER 2 OPENED]")

    #Move to resting position
    resting_joints = [0, 0.64, -1.39, 0, 0, 0]
    n.move_joints(resting_joints)

    n.close_gripper(TOOL_GRIPPER_2_ID, 200)
    print("[GRIPPER 2 CLOSED]")

  n.activate_learning_mode(True)
  moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
  try:
	  niryo_demo()
  except rospy.ROSInterruptException:
	  n.activate_learning_mode(True)

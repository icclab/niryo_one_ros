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
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def test_camera():
	rospy.init_node("camera_test")

	bcaster = tf2_ros.StaticTransformBroadcaster()

	#with open('camera_params.yaml', 'r') as infile:
	#	cam_pose = yaml.load(infile)

	transform_msg = geometry_msgs.msg.TransformStamped()
	transform_msg.header.frame_id = "ground_link"
	transform_msg.child_frame_id  = "camera_optical_frame"
	transform_msg.transform.translation.x = 0.611237 
	transform_msg.transform.translation.y = 0.433886
	transform_msg.transform.translation.z = 0.443974 

        #quat = tf.transformations.quaternion_from_euler(radians(0), radians(125), radians(-144))
        quat_rot = tf.transformations.quaternion_from_euler(-0.070526, 0.697037, -2.089600)
        #quat_rot = tf.transformations.quaternion_multiply(quat,
        #tf.transformations.quaternion_from_euler(0, -pi/6 +pi/18 ,-pi/2 -pi/9))
        
        #quat_rot = tf.transformations.quaternion_multiply(quat,
        #tf.transformations.quaternion_from_euler(0.070526, 1.484623, -0.42367))

        
        print quat_rot

	transform_msg.transform.rotation.x = quat_rot[0]
	transform_msg.transform.rotation.y = quat_rot[1]
	transform_msg.transform.rotation.z = quat_rot[2]
	transform_msg.transform.rotation.w = quat_rot[3]
	bcaster.sendTransform(transform_msg)
	rospy.spin()
	#time.sleep(5)


if __name__ == "__main__":
	test_camera()

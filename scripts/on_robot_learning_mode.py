#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import time

def niryo_demo():
	rospy.init_node('moveit_niryo_robot')

	n = NiryoOne()
	n.activate_learning_mode(True)

if __name__ == '__main__':
	try:
		niryo_demo()
	except rospy.ROSInterruptException:
		n.activate_learning_mode(True)
		

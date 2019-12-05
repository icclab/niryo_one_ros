#!/usr/bin/env python
import rospy
import numpy as np
import copy
import tf
import tf2_ros
import yaml
import datetime
import gc
from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3, Pose, TransformStamped, PointStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
import sensor_msgs.msg #import PointCloud2
from gpd_controller import GpdGrasps
from robot_controller import RobotPreparation
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from tf.transformations import *
from filter_pointcloud_client import call_pointcloud_filter_service
from rosnode import get_node_names, kill_nodes
from pointcloud_operations import create_mesh_and_save
from sensor_msgs import point_cloud2

#import geometry_msgs.msg #for pose2 simple
import math
#from tools import *
#from pprint import pprint
#from pyquaternion import Quaternion
#from gpd.msg import GraspConfigList
#from moveit_python import *
#from moveit_msgs.msg import Grasp, PlaceLocation
#from geometry_msgs.msg import PoseStamped, Vector3, Pose
#from trajectory_msgs.msg import JointTrajectoryPoint
#from visualization_msgs.msg import Marker
#from std_msgs.msg import Header, ColorRGBA
#from moveit_python.geometry import rotate_pose_msg_by_euler_angles
import sys
import moveit_commander
#import moveit_msgs.msg
#import geometry_msgs.msg
#from math import pi
#from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.msg import tfMessage
from niryo_one_python_api.niryo_one_api import *
import time
#import yaml
import subprocess

class GpdPickPlace(object):
    grasps = []
    mark_pose = False
    #grasp_offset = 0
    grasp_offset = -0.03
    #grasp_offset = -0.05

    def __init__(self, mark_pose=False):
        self.grasp_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList,
                                                 self.grasp_callback)
        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
            #self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        self.p = PickPlaceInterface(group="arm", ee_group="tool", verbose=True)
        self.planning = PlanningSceneInterface("camera_color_optical_frame")
        #self.tfBuffer = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.transformer = tf2_ros.BufferInterface()

        self.tf = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #self.bcaster = tf2_ros.StaticTransformBroadcaster()
        #self.transformer = tf.TransformerROS(cache_time = rospy.Duration(10.0))
        #time.sleep(3)

    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasp_subscriber.unregister()
       # frame_id = msg.header.frame_id
        pevent("Received new grasps")

    def show_grasp_pose(self, publisher, grasp_pose):
        # pinfo("Marker orientation:")
        # pprint(grasp_pose.orientation)
        marker = Marker(
            type=Marker.ARROW,
            id=0,
            lifetime=rospy.Duration(30),
            pose=grasp_pose,
            scale=Vector3(0.03, 0.02, 0.02),
            header=Header(frame_id='camera_color_optical_frame'),
            color=ColorRGBA(1.0, 1.0, 0.0, 0.8))
        publisher.publish(marker)

    #def show_grasp_pose(self, publisher, grasp_pose):
    #    place_marker_at_pose(publisher, grasp_pose)

    def place_marker_at_pose(self, publisher, poseStamped):
        marker_x = Marker(
            type=Marker.ARROW,
            id=0,
            lifetime=rospy.Duration(60),
            pose=poseStamped.pose,
            scale=Vector3(0.1, 0.01, 0.01),
            header=poseStamped.header,
            color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        publisher.publish(marker_x)
    
    def get_gpd_grasps(self):
        pevent("Waiting for grasps to arrive")
        while len(self.grasps) == 0:
            rospy.sleep(0.01)
        return self.grasps

    def generate_grasp_msgs(self, grasps):
        formatted_grasps = []
        for i in range(0, len(grasps)):

            g = Grasp()
            g.id = "dupa_" + str(i)
            gp = PoseStamped()
            gp.header.frame_id = "camera_color_optical_frame"


            org_q = self.trans_matrix_to_quaternion(grasps[i])
         #   rot_q = Quaternion(0.7071, 0.7071, 0, 0)  # 90* around X axis (W, X, Y, Z)
          #  quat = rot_q * org_q

            quat = org_q

            # Move grasp back for given offset
            gp.pose.position.x = grasps[i].surface.x + self.grasp_offset * grasps[i].approach.x
            gp.pose.position.y = grasps[i].surface.y + self.grasp_offset * grasps[i].approach.y
            gp.pose.position.z = grasps[i].surface.z + self.grasp_offset * grasps[i].approach.z
        #why this like Lukasz?
            gp.pose.orientation.x = float(quat.elements[1])
            gp.pose.orientation.y = float(quat.elements[2])
            gp.pose.orientation.z = float(quat.elements[3])
            gp.pose.orientation.w = float(quat.elements[0])
            #pprint(gp.pose.orientation)

            g.grasp_pose = gp

            g.pre_grasp_approach.direction.header.frame_id = "tool_link"
            g.pre_grasp_approach.direction.vector.x = 1.0
            g.pre_grasp_approach.direction.vector.y = 0.0
            g.pre_grasp_approach.direction.vector.z = 0.0
            g.pre_grasp_approach.min_distance = 0.04
            g.pre_grasp_approach.desired_distance = 0.08

         #   g.pre_grasp_posture.joint_names = ["joint_6"]
         #   g.pre_grasp_posture.header.frame_id = "hand_link"
         #   pos = JointTrajectoryPoint()
          #  pos.positions.append(0)
            #  pos.positions.append(0.1337)
           # g.pre_grasp_posture.points.append(pos)

          #  g.grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
          #  g.grasp_posture.joint_names = ["joint_6"]
           # pos = JointTrajectoryPoint()
           # pos.positions.append(0.0)
           # pos.positions.append(0.0)
           # pos.accelerations.append(0.0)
           # pos.accelerations.append(0.0)
           # g.grasp_posture.points.append(pos)
           # g.grasp_posture.header.frame_id = "hand_link"

            #g.allowed_touch_objects = ["<octomap>", "obj"]
            g.allowed_touch_objects = ["obj"]
            g.max_contact_force = 0.0
            #g.grasp_quality = grasps[0].score.data  perche 0 e non i????
            g.grasp_quality = grasps[i].score.data

            #Create virtual link so I can get the transform from the gripper_link to grasp pose
            # transform_msg = geometry_msgs.msg.TransformStamped()
            # transform_msg.header.frame_id = "camera_color_optical_frame"
            # transform_msg.child_frame_id  = "virtual_frame"
            # transform_msg.transform.translation.x = g.grasp_pose.pose.position.x
            # transform_msg.transform.translation.y = g.grasp_pose.pose.position.y
            # transform_msg.transform.translation.z = g.grasp_pose.pose.position.z
            # transform_msg.transform.rotation.x = g.grasp_pose.pose.orientation.x
            # transform_msg.transform.rotation.y = g.grasp_pose.pose.orientation.y
            # transform_msg.transform.rotation.z = g.grasp_pose.pose.orientation.z
            # transform_msg.transform.rotation.w = g.grasp_pose.pose.orientation.w
            # self.bcaster.sendTransform(transform_msg)
            # time.sleep(1)

            # #t = self.tf.getLatestCommonTime("virtual_frame", "gripper_link")
            # t = rospy.Time(0)
            # self.tf.waitForTransform("gripper_link", "virtual_frame", t, rospy.Duration(4.0))
            # (v_trans, v_rot) = self.tf.lookupTransformFull("gripper_link", t, "virtual_frame", t, "base_link")

            # #t = self.tf.getLatestCommonTime("tool_link", "base_link")
            # self.tf.waitForTransform("base_link", "tool_link", t, rospy.Duration(4.0))
            # (tool_trans, tool_rot) = self.tf.lookupTransformFull("base_link",t, "tool_link", t, "base_link")

            # pprint((v_trans, tool_trans))

            # #Update the grasp message, tool_link and gripper have the same orientation
            # g.grasp_pose.pose.position.x = tool_trans[0] + v_trans[0]
            # g.grasp_pose.pose.position.y = tool_trans[1] + v_trans[1]
            # g.grasp_pose.pose.position.z = tool_trans[2] + v_trans[2]

            # gp.header.frame_id = "base_link"

            #t = rospy.Time(0)
            #grasp_point = geometry_msgs.msg.PointStamped()
            #grasp_point.header.frame_id = "camera_color_optical_frame"
            #grasp_point.point = g.grasp_pose.pose.position

            #Get grasp point in base_link coordinate system
            #t = self.tf.getLatestCommonTime("camera_color_optical_frame", "base_link")
            #print(t)
            #self.tf.waitForTransform("camera_color_optical_frame", "base_link", t, rospy.Duration(4.0))
            #grasp_base = self.transformer.TransformPose("base_link", grasp_point)
            #grasp_base = self.transformer.transform(grasp_point, "base_link", timeout=rospy.Duration(4.0))
            
            grasp_base = self.tf.transformPose("base_link", g.grasp_pose)

            # #Get tool and gripper translations from base_link
            # #self.tf.waitForTransform("base_link", "tool_link", rospy.Duration(4.0))
            # tool_trans, _    = self.tf.lookupTransform("base_link", "tool_link", rospy.Time(0))
            # gripper_trans, _ = self.tf.lookupTransform("base_link", "gripper_link", rospy.Time(0))

            # g.grasp_pose.header.frame_id = "base_link"
            # g.grasp_pose.pose.position.x = tool_trans[0] + grasp_base.pose.position.x - gripper_trans[0]
            # g.grasp_pose.pose.position.y = tool_trans[1] + grasp_base.pose.position.y - gripper_trans[1]
            # g.grasp_pose.pose.position.z = tool_trans[2] + grasp_base.pose.position.z - gripper_trans[2]
            # g.grasp_pose.pose.orientation.x = grasp_base.pose.orientation.x
            # g.grasp_pose.pose.orientation.y = grasp_base.pose.orientation.y
            # g.grasp_pose.pose.orientation.z = grasp_base.pose.orientation.z
            # g.grasp_pose.pose.orientation.w = grasp_base.pose.orientation.w
            #pprint(g.grasp_pose)

            # q = Quaternion(g.grasp_pose.pose.orientation.w,
            #                g.grasp_pose.pose.orientation.x,
            #                g.grasp_pose.pose.orientation.y,
            #                g.grasp_pose.pose.orientation.z)

            # (x_axis, z_axis) = (q.rotate([1.0, 0.0, 0.0]),
            #                     q.rotate([0.0, 0.0, 1.0]))

            # g.grasp_pose.header.frame_id = "base_link"
            # g.grasp_pose.pose.position.x = grasp_base.pose.position.x - 0.025 * x_axis[0] + 0.015 * z_axis[0] 
            # g.grasp_pose.pose.position.y = grasp_base.pose.position.y - 0.025 * x_axis[1] + 0.015 * z_axis[1] 
            # g.grasp_pose.pose.position.z = grasp_base.pose.position.z - 0.025 * x_axis[2] + 0.015 * z_axis[2] 
            # g.grasp_pose.pose.orientation.x = grasp_base.pose.orientation.x
            # g.grasp_pose.pose.orientation.y = grasp_base.pose.orientation.y
            # g.grasp_pose.pose.orientation.z = grasp_base.pose.orientation.z
            # g.grasp_pose.pose.orientation.w = grasp_base.pose.orientation.w
            t = rospy.Time.now()
            self.br.sendTransform((grasp_base.pose.position.x, grasp_base.pose.position.y, grasp_base.pose.position.z),
                              (grasp_base.pose.orientation.x, grasp_base.pose.orientation.y, grasp_base.pose.orientation.z, grasp_base.pose.orientation.w),
                              t, "grasp_frame", "base_link")

            self.br.sendTransform((-0.025, 0.0, 0.015), (0, 0, 0, 1), t, "virtual_tool", "grasp_frame")

            tool_pose = geometry_msgs.msg.PoseStamped()
            tool_pose.header.frame_id = "virtual_tool"
            tool_pose.pose.orientation.w = 1.0

            self.tf.waitForTransform("base_link", "virtual_tool", t, rospy.Duration(4.0))
            g.grasp_pose.header.frame_id = "base_link"
            g.grasp_pose = self.tf.transformPose("base_link", tool_pose)


            formatted_grasps.append(g)
        return formatted_grasps

    def trans_matrix_to_quaternion(self, grasp):
        r = np.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x],
                      [grasp.approach.y, grasp.binormal.y, grasp.axis.y],
                      [grasp.approach.z, grasp.binormal.z, grasp.axis.z]])
        return Quaternion(matrix=r)

    def pick(self, grasps_list, verbose=False):
        failed_grasps = 0
        pevent("Pick sequence started")

        # Add object mesh to planning scene
        self.add_object_mesh()

        #t = self.tf.getLatestCommonTime("base_link", "camera_depth_optical_frame")


        for single_grasp in grasps_list:
            if self.mark_pose:
                self.show_grasp_pose(self.marker_publisher, single_grasp.grasp_pose.pose)
                rospy.sleep(1)

            #single_grasp.grasp_pose = self.tf.transformPose("base_link", single_grasp.grasp_pose)

            if verbose:
                pevent("Executing grasp: ")
                pprint(single_grasp.grasp_pose)

            self.place_marker_at_pose(self.marker_publisher, single_grasp.grasp_pose)
            pick_result = self.p.pickup("obj", [single_grasp, ], planning_time=9001, support_name="<octomap>",
                                        allow_gripper_support_collision=True)

            pevent("Planner returned: " + get_moveit_error_code(pick_result.error_code.val))

            if pick_result.error_code.val == 1:
                pevent("Grasp successful!")
                return single_grasp
            else:
                failed_grasps += 1
                if failed_grasps == 5:
                    pevent("All grasps failed. Aborting")
                    exit(1)

    def place2(self, place_msg, niryo):
        pevent("Place sequence started")
        group_name = "arm"

        group = moveit_commander.MoveGroupCommander(group_name)

        p2 = copy.deepcopy(place_msg.grasp_pose.pose)
        p2.position.z = 0.3
        group.set_pose_target(p2)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        p3 = copy.deepcopy(place_msg.grasp_pose.pose)
        p3.position.y = -place_msg.grasp_pose.pose.position.y
        p3.position.z = 0.3
        group.set_pose_target(p3)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        p4 = copy.deepcopy(place_msg.grasp_pose.pose)
        p4.position.y = -place_msg.grasp_pose.pose.position.y
        group.set_pose_target(p4)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # group.go(joint_goal, wait=True)

        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        group.clear_pose_targets()

        niryo.open_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 2 opened")

        p5 = copy.deepcopy(place_msg.grasp_pose.pose)
        p5.position.y = -place_msg.grasp_pose.pose.position.y
        p5.position.z = 0.3
        group.set_pose_target(p5)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        # p6 = Pose()
        # p6.position.x = 0.065
        # p6.position.y = 0.0
        # p6.position.z = 0.207
        # p6.orientation.x = 0.0
        # p6.orientation.y = 0.007
        # p6.orientation.z = 0.0
        # p6.orientation.w = 1.0
        # group.set_pose_target(p6)
        # plan = group.go(wait=True)
        # group.stop()
        # group.clear_pose_targets()

    def return_to_rest(self, place_msg):
        pevent("Returning to resting position")
        group_name = "arm"

        group = moveit_commander.MoveGroupCommander(group_name)

        p1 = copy.deepcopy(place_msg.grasp_pose.pose)
        p1.position.y = -place_msg.grasp_pose.pose.position.y
        p1.position.z = 0.3
        group.set_pose_target(p1)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        p2 = Pose()
        p2.position.x = 0.065
        p2.position.y = 0.0
        p2.position.z = 0.207
        p2.orientation.x = 0.0
        p2.orientation.y = 0.007
        p2.orientation.z = 0.0
        p2.orientation.w = 1.0
        group.set_pose_target(p2)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    def place(self, place_msg):
        pevent("Place sequence started")

        #places = self.generate_place_poses(place_pose)
        #place_pose is a Grasp msg
        l = PlaceLocation()
        l.id = "place target"
        l.place_pose = place_msg.grasp_pose
        l.place_pose.pose.position.y = -l.place_pose.pose.position.y

        _, place_result = self.p.place_with_retry("obj", [l, ], support_name="<octomap>", planning_time=9001,
                                  goal_is_eef=True)

        pevent("Planner returned: " + get_moveit_error_code(place_result.error_code.val))
        #Keep getting INVALID_GROUP_NAME - why???
        #Does pick kill the planning group?

    def generate_place_poses(self, initial_place_pose):
        places = list()

        l = PlaceLocation()
        l.id = "dupadupa"
        l.place_pose.header.frame_id = "camera_color_optical_frame"


        q = Quaternion(initial_place_pose.grasp_pose.pose.orientation.w,
                        initial_place_pose.grasp_pose.pose.orientation.x,
                        initial_place_pose.grasp_pose.pose.orientation.y,
                        initial_place_pose.grasp_pose.pose.orientation.z)

# Load successful grasp pose
        l.place_pose.pose.position = initial_place_pose.grasp_pose.pose.position
        l.place_pose.pose.orientation.w = q.elements[0]
        l.place_pose.pose.orientation.x = q.elements[1]
        l.place_pose.pose.orientation.y = q.elements[2]
        l.place_pose.pose.orientation.z = q.elements[3]

# Move 20cm to the right
        l.place_pose.pose.position.y += 0.2

# Fill rest of the msg with some data
        l.post_place_posture = initial_place_pose.grasp_posture
        l.post_place_retreat = initial_place_pose.post_grasp_retreat
        l.pre_place_approach = initial_place_pose.pre_grasp_approach

        places.append(copy.deepcopy(l))

# Rotate place pose to generate more possible configurations for the planner
        m = 16  # Number of possible place poses
        for i in range(0, m - 1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * math.pi / m)
            places.append(copy.deepcopy(l))

        return places

    def add_object_mesh(self):
        obj_pose = Pose()
        obj_pose.position.x = 0
        obj_pose.position.y = 0
        obj_pose.position.z = 0
        obj_pose.orientation.x = 0
        obj_pose.orientation.y = 0
        obj_pose.orientation.z = 0
        obj_pose.orientation.w = 1
        self.planning.addMesh("obj", obj_pose, "object.stl", use_service = True)

        # p = Pose()
        # p.position.x = 0
        # p.position.y = 0
        # p.position.z = 0
        # p.orientation.x = 0
        # p.orientation.y = 0
        # p.orientation.z = 0
        # p.orientation.w = 1
        # planning.add_box("table", p, (1, 1, 1))
        #t = rospy.Time.now()
        #self.tf.waitForTransform("ground_link", "camera_color_optical_frame", t, rospy.Duration(4.0))
        #(trans, quat) = self.tf.lookupTransform("camera_color_optical_frame", "ground_link", t)

        self.planning.attachBox("table", 1, 1, 1, 0, 0, -0.5, link_name="ground_link")
        #time.sleep(15)
        #planning.clear()
     #   rospy.sleep(3.14)
     #   pprint(planning.getKnownCollisionObjects())

    def get_know_successful_grasp(self):
        g = Grasp()
        g.id = "successful_predefined_grasp"
        gp = PoseStamped()
        gp.header.frame_id = "camera_color_optical_frame"

        gp.pose.position.x = 0.183518647951
        gp.pose.position.y = -0.23707952283
        gp.pose.position.z = 0.493978534979

        gp.pose.orientation.w = -0.604815599864
        gp.pose.orientation.x = -0.132654186819
        gp.pose.orientation.y = 0.698958888788
        gp.pose.orientation.z = -0.357851126398

        g.grasp_pose = gp

        return g

    def wait_for_mesh_and_save(self):
      pinfo("Subscribing to pointcloud to generate mesh")
      self.obj_pc_subscriber = rospy.Subscriber("/cloud_indexed_pc_only", sensor_msgs.msg.PointCloud2 , self.obj_pointcloud_callback)
      

    def obj_pointcloud_callback(self, msg): #msg is a sensor_msgs.msg.PointCloud2
#      pcl::toROSMsg
      pinfo("Pointcloud received")
      cloud = []
      for p in point_cloud2.read_points(msg, skip_nans=True):
                cloud.append([p[0], p[1], p[2]])
      create_mesh_and_save(cloud)
      pinfo("Mesh generated")
      self.obj_pc_subscriber.unregister()



if __name__ == "__main__":
    start_time = datetime.datetime.now()
    rospy.init_node("gpd_pick_and_place")

    print("--- Start Physical Arm ---")

    #n = NiryoOne()

 #   print("Calibration started !")
    # Calibrate robot first
#try:
 #   n.calibrate_auto()
#except NiryoOneException as e:
#    print e

    print("Make sure calibration is already performed on arm !")
    time.sleep(1)
    # Test learning mode
    #n.activate_learning_mode(False)

    # Test gripper 3
    #n.change_tool(TOOL_GRIPPER_3_ID)

    # testing to add a box at the eef to simulate a gripper
 #   robot = moveit_commander.RobotCommander()
 #   scene = moveit_commander.PlanningSceneInterface()
 #   group_name = "arm"
 #   group = moveit_commander.MoveGroupCommander(group_name)
    # We can get the name of the reference frame for this robot:
#    planning_frame = group.get_planning_frame()
#    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
#    eef_link = group.get_end_effector_link()
#    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
#    group_names = robot.get_group_names()
#    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
#    print("============ Printing robot state")
#    print(robot.get_current_state())
#    print("")

    #Read camera-to-ground transform from yaml file and publish
    # with open('camera_params.yaml', 'r') as infile:
    #     cam_pose = yaml.load(infile)

    # camera_static_transform = subprocess.Popen(
    #             ["rosrun", "tf", "static_transform_publisher", str(cam_pose[0]), str(cam_pose[1]), str(cam_pose[2]), 
    #             str(cam_pose[3]), str(cam_pose[4]), str(cam_pose[5]), str(cam_pose[6]), "camera_color_optical_frame", "ground_link", "100"])

    # print("[INFO] Camera-robot link established")

    num_objects = 1
    for i in range (0, num_objects):
        # Subscribe for grasps
        pnp = GpdPickPlace(mark_pose=True)
        pnp.planning.clear()

         # Get the pointcloud from camera, filter it, extract indices and publish it to gpd CNN
        #gpd_prep = GpdGrasps(max_messages=8)
        #gpd_prep.filter_cloud()
        #gpd_prep.publish_indexed_cloud()

        call_pointcloud_filter_service()
        pnp.wait_for_mesh_and_save()

        # Wait for grasps from gpd, wrap them into Grasp msg format and start picking
        selected_grasps = pnp.get_gpd_grasps()
        formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
        n.open_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 2 opened")
        successful_grasp = pnp.pick(formatted_grasps, verbose=False)
        n.close_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 2 closed")


        # Place object with successful grasp pose as the starting point
        pnp.place2(successful_grasp, n)
        #n.open_gripper(TOOL_GRIPPER_3_ID, 200)
        #print("Gripper 2 opened")
        #pnp.return_to_rest(successful_grasp)
        #n.close_gripper(TOOL_GRIPPER_3_ID, 200)
        #print("Gripper 2 closed")
  #  pnp.place(successful_grasp)
  #  fix_grasp =  pnp.get_know_successful_grasp()
   # pnp.place(fix_grasp)
    pinfo("Demo runtime: " + str(datetime.datetime.now() - start_time))

    #static_transform_nodes = [node_name for node_name in get_node_names() if node_name[:27] == '/static_transform_publisher']
    #kill_nodes(static_transform_nodes)

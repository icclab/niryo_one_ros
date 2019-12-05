#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2
import tf

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from math import pi
#from transform_point_cloud.cfg import LookupTransformConfig


class TransformPointCloud:
    def __init__(self):
        self.config = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("point_cloud_transformed", PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber("/camera/depth/points", PointCloud2,
                                    self.point_cloud_callback, queue_size=2)
        #self.dr_server = Server(LookupTransformConfig, self.dr_callback)

#    def dr_callback(self, config, level):
#        self.config = config
#        return self.config

    def point_cloud_callback(self, msg):
        target_frame = "ground_link" 
        source_frame = "camera_link" 
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0),
                                                    rospy.Duration(10))
            # print trans
            #print trans.transform.rotation.x
            quat = [0, 0, 0, 0]
            quat[0] = trans.transform.rotation.x
            quat[1] = trans.transform.rotation.y
            quat[2] = trans.transform.rotation.z
            quat[3] = trans.transform.rotation.w

            quat_rot = tf.transformations.quaternion_multiply(quat,
            tf.transformations.quaternion_from_euler(0,-pi/6+pi/18 ,-pi/2 -pi/9))
            
            trans.transform.translation.x = trans.transform.translation.x - 0.2
            trans.transform.translation.y = trans.transform.translation.y + 0.1

            trans.transform.rotation.x = quat_rot[0]
            trans.transform.rotation.y = quat_rot[1]
            trans.transform.rotation.z = quat_rot[2]
            trans.transform.rotation.w = quat_rot[3]


            print "stuff Transformed"
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(msg, trans)
        self.pub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    rospy.spin()

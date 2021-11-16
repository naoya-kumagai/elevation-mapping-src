#!/usr/bin/env python3
# http://wiki.ros.org/tf2/Tutorials/Quaternions
#https://github.com/IntelRealSense/realsense-ros/blob/bb5c713d57c41c30609862591d778f4c33abd695/realsense2_camera/urdf/mount_t265_d435.urdf.xacro

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import time

class ROSController:

    def __init__(self):
       # Obtain odom values to create transformation frame between base and world
       self.sub_tf_world = rospy.Subscriber('/cam_tracking/odom/sample', Odometry, self.Callback_tf_world)
       
       self.q = quaternion_from_euler(0.000, -0.018, 0.005)
       self.current = time.time()
       
    def Callback_tf_world(self, msg):
        # this function will broadcast the world frame relative to the base frame. This is required if we want to do
        # mapping or visualize for rviz applications
        tf_world_broadcaster = tf2_ros.TransformBroadcaster()
        tf_world = geometry_msgs.msg.TransformStamped()
        #tf_world.header.stamp = rospy.Time.now()
        tf_world.header.stamp = rospy.Time(0)
        #tf_world.header.frame_id = "odom"
        tf_world.header.frame_id = "map"
        tf_world.child_frame_id = "cam_depth_link"
        tf_world.transform.translation.x = 0.009	
        tf_world.transform.translation.y = 0.021             
        tf_world.transform.translation.z = 0.027
        tf_world.transform.rotation.x = self.q[0]
        tf_world.transform.rotation.y = self.q[1]
        tf_world.transform.rotation.z = self.q[2]
        tf_world.transform.rotation.w = self.q[3]
   
   
        tf_world_broadcaster.sendTransform(tf_world) 
        
        print(time.time() - self.current)
        self.current = time.time()
        
        
if __name__ == '__main__':
  rospy.init_node('odom_to_trackingcam_tf_broadcaster')
  
  roscontroller = ROSController()
  rospy.spin()
   

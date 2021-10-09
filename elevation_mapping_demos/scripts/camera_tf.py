import rospy
import tf
import tf2_ros
import geometry_msgs.msg



def Callback_tf_world(self, msg):
        # this function will broadcast the world frame relative to the base frame. This is required if we want to do
        # mapping or visualize for rviz applications
        tf_world_broadcaster = tf2_ros.TransformBroadcaster()
        tf_world = geometry_msgs.msg.TransformStamped()
        tf_world.header.stamp = rospy.Time.now()
        tf_world.header.frame_id = "world"
        tf_world.child_frame_id = "base"
        tf_world.transform.translation.x = msg.pose.pose.position.x
        tf_world.transform.translation.y = msg.pose.pose.position.y
        tf_world.transform.translation.z = msg.pose.pose.position.z
        tf_world.transform.rotation.x = msg.pose.pose.orientation.x
        tf_world.transform.rotation.y = msg.pose.pose.orientation.y
        tf_world.transform.rotation.z = msg.pose.pose.orientation.z
        tf_world.transform.rotation.w = msg.pose.pose.orientation.w
        tf_world_broadcaster.sendTransform(tf_world) 
        
if __name__ == '__main__':
  rospy.init_node('camera_tf')
  # Obtain odom values to create transformation frame between base and world
self.sub_tf_world = rospy.Subscriber('/a1_gazebo/odom', Odometry, self.Callback_tf_world)

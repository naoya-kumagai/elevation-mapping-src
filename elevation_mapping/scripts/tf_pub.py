#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


# def handle_pose():
#     rospy.loginfo('Received data')
 
#     br = tf2_ros.TransformBroadcaster()
#     t = geometry_msgs.msg.TransformStamped()

#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = "/map"
#     t.child_frame_id = "/cam_tracking_odom_frame"
#     t.transform.translation.x = 0
#     t.transform.translation.y = 0
#     t.transform.translation.z = 0
#     q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
#     t.transform.rotation.x = q[0]
#     t.transform.rotation.y = q[1]
#     t.transform.rotation.z = q[2]
#     t.transform.rotation.w = q[3]

#     br.sendTransform(t)

  

# if __name__ == '__main__':
#     rospy.init_node('tf2_broadcaster_test')
#     sub = rospy.Subscriber('plane_tf_info', 
#                      Odometry,
#                      sub_test
#                     )
#     print(sub)
#     while not rospy.is_shutdown():
#         handle_pose()
#     # turtlename = rospy.get_param('~turtle')

#     print(sub)
#     rospy.spin()

class tf_handle(object):
    def __init__(self):
        self.plane_converged = False

        rospy.Subscriber('plane_tf_info', Odometry, self.sub_test)
        rospy.Subscriber('converged_info', Bool, self.set_convergence_flag)

        self.t = geometry_msgs.msg.TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
    
    def set_convergence_flag(self, data):
        self.plane_converged = data
               

    def sub_test(self, data):
        print('Entered sub_test!')

        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "/map"
        self.t.child_frame_id = "/cam_tracking_odom_frame"
        self.t.transform.translation.x = 1.0
        self.t.transform.translation.y = 0
        self.t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0.78, 0)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

        self.br.sendTransform(self.t)

    def default_tf(self):

        print('Entered default')
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "/map"
        self.t.child_frame_id = "/cam_tracking_odom_frame"
        self.t.transform.translation.x = 0
        self.t.transform.translation.y = 0
        self.t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

        self.br.sendTransform(self.t)


    

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_test')

    subandpub = tf_handle()
    while not subandpub.plane_converged:
        subandpub.default_tf()

    rospy.spin()


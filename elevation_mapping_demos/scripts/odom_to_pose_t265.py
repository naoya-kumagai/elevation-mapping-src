#!/usr/bin/env python3

import rospy
import geometry_msgs.msg 
from nav_msgs.msg import Odometry
    


class OdometryModifier:

    def __init__(self):
        self.sub = rospy.Subscriber("/cam_tracking/odom/sample", Odometry, self.callback)
        self.pub = rospy.Publisher('/camera_footprint_pose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)               

    def callback(self, odom):
        #rospy.loginfo('odom callback!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'cam_tracking_pose_frame' ####FIXME##########
 
        #pose.pose.pose = odom.pose.pose
       
        pose.pose.pose.position.x = odom.pose.pose.position.x
        pose.pose.pose.position.y = odom.pose.pose.position.y
        pose.pose.pose.position.z = odom.pose.pose.position.z #what was this +1.0 for???
       
        #pose.pose.pose.position.z = 0.1
      
        pose.pose.pose.orientation.x = odom.pose.pose.orientation.x
        pose.pose.pose.orientation.y = odom.pose.pose.orientation.y
        pose.pose.pose.orientation.z = odom.pose.pose.orientation.z
        pose.pose.pose.orientation.w = odom.pose.pose.orientation.w
        
        pose.pose.covariance = odom.pose.covariance
        # print(odom.pose.covariance)


        self.pub.publish(pose)

if __name__ == '__main__':
    rospy.loginfo('entered main')
    try:
        rospy.init_node('tf_to_pose_publisher')
        odom = OdometryModifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

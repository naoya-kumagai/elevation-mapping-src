#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(1.0)
    from_frame = 'map'
    to_frame = 'cam_tracking_pose_frame'
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform(
            from_frame, to_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
            continue
        
        
        
    
    
        br.sendTransform((0.0, 0.0, 0.3),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "turtle1",
                         "map"
                         )
        #rate.sleep()

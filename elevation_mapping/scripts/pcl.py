from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import rospy
import pickle
pcls = []

def callback(ros_point_cloud):
    xyz = np.array([[0,0,0]])
    rgb = np.array([[0,0,0]])
    gen = list(pc2.read_points(ros_point_cloud, skip_nans=True))
    pcls.append(gen)
    
    # for x in int_data:
    #     test = x[3] 
    #     # cast float32 to int so that bitwise operations are possible
    #     s = struct.pack('>f' ,test)
    #     i = struct.unpack('>l',s)[0]
    #     # you can get back the float value by the inverse operations
    #     pack = ctypes.c_uint32(i).value
    #     r = (pack & 0x00FF0000)>> 16
    #     g = (pack & 0x0000FF00)>> 8
    #     b = (pack & 0x000000FF)
    #     # prints r,g,b values in the 0-255 range
    #                 # x,y,z can be retrieved from the x[0],x[1],x[2]
    #     xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
    #     rgb = np.append(rgb,[[r,g,b]], axis = 0)


    print(xyz)

try:
    rospy.init_node('listener1', anonymous=True)
    cloud_sub = rospy.Subscriber("/cam_depth/depth/color/points", PointCloud2,callback,queue_size=1, buff_size=52428800)      
    rospy.spin()
finally:
    with open('pcl.pkl', 'wb') as handle:
        pickle.dump(pcls, handle)
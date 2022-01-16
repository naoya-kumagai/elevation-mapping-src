
# from IPython.utils.dir2 import dir2
# from numpy.lib.shape_base import get_array_wrap
from grid_map_msgs.msg import GridMap
import rospy
from std_msgs.msg import String
import numpy as np

import cv2
import sys
import os


import time
# from skimage.measure import find_contours
# import matplotlib.pyplot as plt
# from matplotlib import patches,  lines
# from matplotlib.patches import Polygon


def wrap_in_size(size_x,size_y,ind_x,ind_y):
    if ind_x<0:
        ind_x += (((-ind_x) // size_x) + 1) * size_x
    if ind_y<0:
        ind_y += (((-ind_y) // size_y) + 1) * size_y
    return ind_x%size_x, ind_y%size_y	



def move_array(ind_x,ind_y,a):
    b = np.array(a)
    for i in range(a.shape[0]):
        for j in range(a.shape[1]):
            new_ind_x = i + ind_x
            new_ind_y = j + ind_y
            new_ind_x, new_ind_y = wrap_in_size(a.shape[0],a.shape[1],new_ind_x,new_ind_y)
            b[i,j]=a[new_ind_x, new_ind_y] 
    return b


def translate_matrix(ind_x,ind_y,a):
    d0 = np.array(a.data).reshape((a.layout.dim[0].size,a.layout.dim[1].size))
    d0 = np.transpose(d0)
    d0 = move_array(ind_x,ind_y,d0)
    return d0

class elevation_map_listener(object):

    def __init__(self):

        self.robot_pose = None
        self.resolution = None
        self.length_x = None
        self.length_y = None
        self.height_map = None #2D numpy array
        self.color_map = None
        self.height_map_filtered = None
        # self.height_map_plane = None
        self.uncertainty_range_map = None
        self.accumulated_height_map = None

        self.surface_normals_x = None
        self.surface_normals_y = None
        self.surface_normals_z = None
        rospy.init_node('listeneer', anonymous=True)
        #rospy.Subscriber("/grid_map_filter_demo/filtered_map", GridMap, self.callback)		
        rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, self.callback)		
        #rospy.Subscriber("/elevation_mapping_plane/elevation_map", GridMap, self.callback_plane)	
    #update the height map and the surface_normals
    def callback(self,gmdata):
        #print("entered visualization callback")
        # print(gmdata.info)
        # print("++++++++++++++++")
        #print(gmdata.layers)
        # print("---------------")
        #print(len(gmdata.layers))
        # print(gmdata.layers[11])
        # print(gmdata.layers[12])
        # print(gmdata.layers[13])
        idx_elevation = gmdata.layers.index('elevation') 
        #idx_color = gmdata.layers.index('color')
        #idx_uncertainty = gmdata.layers.index('uncertainty_range')
        

        self.robot_pose = gmdata.info.pose
        self.resolution = gmdata.info.resolution
        self.length_x = gmdata.info.length_x
        self.length_y = gmdata.info.length_y
        d = gmdata.data
        #print(len(d))
        self.height_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_elevation])
        #self.uncertainty_range_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_uncertainty])
        #uncertainty_range is calculated as upper_bound - lower_bound
        # upper_bound = elevation + 99th percentile of the "upper bound distribution"
        # lower_bound = elevation - 1st percentile of the "lower bound distribution"

        #print(np.nanmin(self.height_map))
        #self.height_map_filtered = np.where(np.isnan(self.height_map), np.nanmedian(self.height_map), self.height_map)

        # add data to accumulated height map if uncertainty range < threshold 



        #print(self.height_map.shape)
        #self.color_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_color])

        # print(d[idx_color])
        # print("-------------------------------")

            
    
        #self.surface_normals_x = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[11])
        #self.surface_normals_y = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[12])
        #self.surface_normals_z = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[13])

    # def callback_plane(self,gmdata):
    #     #print('entered plane viz callback')
    #     d = gmdata.data
    #     self.height_map_plane = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[0])


    def get_height_r(self,x,y,mode=1):  #In grid_map the map frame has always no rotation relative to the world frame. In this function x and y are in the robot frame.
    #if mode = 0 returns height, if mode = 1 returns surface normals, if mode = 2 returns height and surface normals.
        if x>self.length_x/2 or x<-self.length_x/2 or y>self.length_y/2 or y<-self.length_y/2:
            print("This point is out of the map!!!!")
            return None

        ind_x = x//self.resolution
        if x<0:
            ind_x = ind_x+1
        ind_x = -ind_x

        ind_y = y//self.resolution
        if y<0:
            ind_y = ind_y+1
        ind_y = -ind_y

        if mode == 0:
            return self.height_map[int(ind_x+self.height_map.shape[0]//2),int(ind_y+self.height_map.shape[1]//2)]
        elif mode == 1:
            return [self.surface_normals_x[int(ind_x+self.surface_normals_x.shape[0]//2),int(ind_y+self.surface_normals_x.shape[1]//2)],
                    self.surface_normals_y[int(ind_x+self.surface_normals_y.shape[0]//2),int(ind_y+self.surface_normals_y.shape[1]//2)],
                    self.surface_normals_z[int(ind_x+self.surface_normals_z.shape[0]//2),int(ind_y+self.surface_normals_z.shape[1]//2)]]
        else:
            return self.height_map[int(ind_x+self.height_map.shape[0]//2),int(ind_y+self.height_map.shape[1]//2)],[self.surface_normals_x[int(ind_x+self.surface_normals_x.shape[0]//2),int(ind_y+self.surface_normals_x.shape[1]//2)],
                    self.surface_normals_y[int(ind_x+self.surface_normals_y.shape[0]//2),int(ind_y+self.surface_normals_y.shape[1]//2)],
                    self.surface_normals_z[int(ind_x+self.surface_normals_z.shape[0]//2),int(ind_y+self.surface_normals_z.shape[1]//2)]]

    def get_height_w(self,x,y,mode=1):  #this function assumes that the robot has no rotation relative to the world frame. In this function x and y are in world frame.
        return self.get_height_r(x-self.robot_pose.position.x, y-self.robot_pose.position.y,mode)




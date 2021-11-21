#!/usr/bin/env python3

from elevation_map_sub import listener
# from IPython.utils.dir2 import dir2
# from numpy.lib.shape_base import get_array_wrap
# from grid_map_msgs.msg import GridMap
import rospy
# from std_msgs.msg import String
import numpy as np
import numpy.ma as ma
import cv2
# import sys
# import os
# import time
import matplotlib.pyplot as plt
#from collections import Counter

def height_map_plotter(fig, ax, height_map, resolution, cmap, first_loop, vmin, vmax):
    extent = [0, height_map.shape[1]*resolution, 0, height_map.shape[0]*resolution]
    if first_loop:
        ax.grid()
        pos = ax.imshow(height_map,cmap=cmap, vmin =vmin, vmax =vmax, extent=extent)
        cbar = fig.colorbar(pos, ax=ax, shrink=0.5)
        ax.set_title('Elevation map')
        ax.set_xlabel('map width (m)')
        ax.set_ylabel('map length (m)')
    
    else:
        ax.imshow(height_map,cmap=cmap, vmin =vmin, vmax =vmax, extent=extent)

def plot_height_dist(fig, ax, height_map, first_loop):
    if first_loop:
        pass

    ax.clear()
    ax.set_title('Elevation distribution')
    ax.set_ylabel('number of cells')
    ax.set_xlabel('elevation (m)')
    #ax.set_xlim(-0.44, -0.41) #for plane only
    ax.set_xlim(-0.44, -0.35)
    ax.hist(np.reshape(height_map, (-1,1)), bins = 100)

def get_plane_height_mean(height_map):
    min = -0.44
    max = -0.34
    hist, bin_edges = np.histogram(np.reshape(height_map, (-1,1)),bins=np.linspace(min,max, 100), density=False)
    plane_height_mean = bin_edges[np.argmax(hist)]
    return plane_height_mean

def detect_from_dist(fig, ax, height_map, plane_height_mean, resolution, cmap, first_loop, vmin, vmax):
    '''
    Creates masking of height map based on the distribution of the data.
    This assumes that the plane is the dominant surface area, which is reasonable for the current climbing wall.
    '''
    variance = 0.005 # assumes variance of plane data is 5mm. TODO: improve this!

    plane_height_upper_bound = plane_height_mean + variance

    height_map_masked = np.where(height_map > plane_height_upper_bound, height_map, 0 )

    extent = [0, height_map.shape[1]*resolution, 0, height_map.shape[0]*resolution]
    if first_loop:
        ax.grid()
        pos = ax.imshow(height_map_masked,cmap=cmap, vmin =vmin, vmax =vmax, extent=extent)
        cbar = fig.colorbar(pos, ax=ax, shrink=0.5)
        ax.set_title('Masked elevation map')
        ax.set_xlabel('map width (m)')
        ax.set_ylabel('map length (m)')
    
    else:
        ax.imshow(height_map_masked,cmap=cmap, vmin =vmin, vmax =vmax, extent=extent)
    



def online_plotting(save_map):
    plt.style.use('ggplot')

    my_listener = listener() #the height map would be a top-down view

    fig, (ax1, ax2,ax3) = plt.subplots(nrows=1, ncols=3, figsize=(15,15), constrained_layout=True)
    # model, dataset, config = inference_init(ROOT_DIR=ROOT_DIR, HOLD_DIR='/home/yusuke/Mask_RCNN/hold/dataset2', 
    #     subset='val',weights_path='/home/yusuke/Mask_RCNN/logs/mask_rcnn_hold_0010.h5')
 
    cmap = plt.cm.rainbow
    cmap.set_under('black') # set ubobserved area to -99
    cmap.set_over('white')     # set planar region to 99
    # cmap.set_bad('gray')


    size = 100
    x_vec = np.linspace(0,1,size+1)[0:-1] #0.0, 0.01, 0.02, ... 0.99
    y_vec = np.zeros(len(x_vec))
    line1 = []
    first_loop = True

    true_a = 0.05
    true_b = 0.03
    
    i = 0
            
    while not rospy.is_shutdown():
        #rospy.sleep(0.1)
        # rospy.loginfo('hello')

        if my_listener.height_map is not None: #and my_listener.height_map_plane is not None:
        #if my_listener.uncertainty_range_map is not None:               

            # cax = plt.imshow(height_map_combined, cmap=cmap, vmin = vmin, vmax=vmax)
            # fig.colorbar(cax)
            if first_loop:
                #initialize accumulated height map with 0s
                my_listener.accumulated_height_map = np.zeros(my_listener.height_map.shape)



            #if uncertainty is less than threshold, update that part of the accumulated height map 

            #my_listener.accumulated_height_map = np.where(my_listener.uncertainty_range_map < 0.01, my_listener.height_map, my_listener.accumulated_height_map)

            #dont change!!!!
            vmin = -0.415 - 0.05
            vmax = vmin + 0.10
            vmax=0

            # vmin = None
            # vmax = None

            height_map_plotter(fig, ax1, my_listener.height_map, my_listener.resolution, cmap, first_loop, vmin, vmax)

            plot_height_dist(fig, ax2, my_listener.height_map, first_loop)

            plane_height_mean = get_plane_height_mean(my_listener.height_map)

            detect_from_dist(fig, ax3,my_listener.height_map, plane_height_mean, my_listener.resolution, cmap, first_loop, vmin, vmax)

            if save_map:
                # np.save("/home/yusuke/height_map.npy", my_listener.height_map)
                # print('saved npy file')
                # gray= cv2.normalize(src=my_listener.height_map, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
              
                # gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                
                # hold_num = 'SEllipsoid'
                # dataset_num = '_test3'
                # angle = ''
                # cv2.imwrite(f'/home/yusuke/Mask_RCNN/hold/dataset{dataset_num}/hold_{hold_num}_{angle}{i}.png', gray)
                #np.save(f'/home/yusuke/Mask_RCNN/hold/evaluation3/hold15_dist50cm_{i}.npy', my_listener.height_map)
                #plt.imsave(f'/home/yusuke/Mask_RCNN/hold/evaluation3/hold7_dist50cm_{i}.png', my_listener.height_map, cmap=cmap)
                #cv2.imwrite(f'/home/yusuke/Mask_RCNN/hold/evaluation_3/hold7_dist50cm_{i}.png', gray)
                print('saved image')


            # height_map_plotter(fig, ax1, my_listener.height_map_filtered, cmap, first_loop, vmin, vmax)
            # mask = np.zeros(my_listener.height_map.shape)
            #masked = np.ma.masked_where(((vmin < my_listener.height_map) & (my_listener.height_map < vmax)), my_listener.height_map)
            
            #
            # ax2.imshow(masked, cmap=cmap)

            #contours, edges = edge_detect(ax2, my_listener.height_map_filtered)

            
            #ax2.imshow(edges)
            #x3.imshow(mask)
        
            # contour, ellipse_list = plot_ellipse(my_listener.height_map)
                      
            # ax2.imshow(contour)
            # ax2.set_title('contour map')
            
            # if ellipse_list != []:
            #     (cx, cy), (d1, d2), angle = ellipse_list[0]
            #     # cx, cy = np.array(ellipse_list[0][0], dtype=int)
            #     # ellipse_axis =  ellipse_list[0][1]
            #     # a = max(ellipse_axis)
            #     # b = min(ellipse_axis)
            #     # angle = ellipse_list[0][2]
                
            #     a = max(d1, d2) / 2 * my_listener.resolution 
            #     b = min(d1, d2) / 2 * my_listener.resolution 

            #     y_vec[-1] = a
            
            # line1 = size_plotter(fig, ax3, x_vec,y_vec,line1,true_value=true_a) #updates graph
            # y_vec = np.append(y_vec[1:],0.0)   #append any value to keep y length

            first_loop = False
            i += 1
            plt.pause(0.1) #same as plt.show(), kind of
            #plt.clf()
           

        #print(my_listerner.get_height_w(-2.0,1.0,2))
        else:
            rospy.loginfo("Waiting for height map...")
            rospy.sleep(1.0)    
            



if __name__ == '__main__':
    # print(os.path.dirname(sys.executable))
    # print(sys.version)
    # print(tf.__version__)
    online_plotting(False)
    #offline_test()






def offline_test():
    # fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3)
    # ax1.grid()
    # ax2.grid()
    # ax3.grid()
    # fig.tight_layout() 
    # cmap = plt.cm.rainbow
    #height_map = np.load("/home/yusuke/height_map.npy")
    #gray= cv2.normalize(src=height_map, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    gray = cv2.imread('/home/yusuke/Mask_RCNN/holds/img_30.png')
    
    plt.imshow(gray)
    #height_map_plotter(fig, ax1, height_map, cmap, first_loop=True)

    #contours, edges = edge_detect(ax2, height_map)



    
    # ax2.imshow(edges)
    # ax3.imshow(mask)
    plt.show()
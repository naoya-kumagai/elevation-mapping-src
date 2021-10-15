#!/usr/bin/env python3

from IPython.utils.dir2 import dir2
from grid_map_msgs.msg import GridMap
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import cv2
# import warnings

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

class listener(object):

    def __init__(self):

        self.robot_pose = None
        self.resolution = None
        self.length_x = None
        self.length_y = None
        self.height_map = None #2D numpy array
        self.color_map = None
        # self.height_map_plane = None

        self.surface_normals_x = None
        self.surface_normals_y = None
        self.surface_normals_z = None
        rospy.init_node('listeneer', anonymous=True)
        #rospy.Subscriber("/grid_map_filter_demo/filtered_map", GridMap, self.callback)		
        rospy.Subscriber("/elevation_mapping/elevation_map", GridMap, self.callback)		
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
        idx_color = gmdata.layers.index('color')
        

        self.robot_pose = gmdata.info.pose
        self.resolution = gmdata.info.resolution
        self.length_x = gmdata.info.length_x
        self.length_y = gmdata.info.length_y
        d = gmdata.data
        #print(len(d))
        self.height_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_elevation])
        #print(self.height_map.shape)
        self.color_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_color])

        # print(d[idx_color])
        # print("-------------------------------")

            
    
        #self.surface_normals_x = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[11])
        #self.surface_normals_y = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[12])
        #self.surface_normals_z = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[13])

    def callback_plane(self,gmdata):
        #print('entered plane viz callback')
        d = gmdata.data
        self.height_map_plane = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[0])


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


def plot_ellipse(height_map):
    '''
    m3: contour map with fitted ellipses overlayed
    ellipse_list: list of ellipse data
       each ellipse has length 3: [cx, cy], [b,a], angle
    '''
    # print(height_map.shape)
    mask = np.where(np.isnan(height_map), 0, 255).astype(np.uint8)
    ret, thresh = cv2.threshold(mask, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, 1, 2)
    m3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    has_ellipse = len(contours) > 0
    ellipse_list = []
    if has_ellipse:
        for cnt in contours: 
            try: 
                ellipse = cv2.fitEllipse(cnt)
            except:
                continue
            
            ellipse_list.append(ellipse)
            #plot center
            # cx, cy = np.array(ellipse[0], dtype=int)
            # m3[cy-2:cy+2,cx-2:cx+2] = (255, 0, 0)
            cv2.ellipse(m3, ellipse, (0,0,255), 1)
    else:
        rospy.loginfo("no ellipse detected!")
    
    return m3, ellipse_list

def size_plotter(fig, ax, x_vec,y1_data,line1, true_value, identifier=''):
    '''
    plot estimated size and true size of object 
    '''
    if line1==[]: #called initially
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = fig
        ax = ax

        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)

        # plot true value
        ax.plot(x_vec, true_value*np.ones(len(x_vec)))
        
        #update plot label/title
        ax.set_ylabel('Y Label')
        ax.set_title('Semi major axis')
        plt.show()

    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    # adjust limits if new data goes beyond bounds
    if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])


    # return line so we can update it again in the next iteration
    return line1

def height_map_plotter(fig, ax, height_map, cmap, first_loop):
    if first_loop == True:
        pos = ax.imshow(height_map,cmap=cmap)
        cbar = fig.colorbar(pos, ax=ax)
        #ax.set_ylabel('Y Label')
        ax.set_title('Height map')
    
    else:
        ax.imshow(height_map,cmap=cmap)

    
            

if __name__ == '__main__':
    plt.style.use('ggplot')
    # warnings.filterwarnings("error")
    my_listener = listener() #the height map would be a top-down view

    #fig = plt.figure(figsize=(4,8))
    # fig, (ax1, ax2) = plt.subplots(figsize=(4,8),nrows=1, ncols=2)
    

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3)
    ax1.grid()
    ax2.grid()
    fig.tight_layout() 
    cmap = plt.cm.rainbow
    cmap.set_under('gray') # set ubobserved area to -99
    cmap.set_over('white')     # set planar region to 99

    size = 100
    x_vec = np.linspace(0,1,size+1)[0:-1] #0.0, 0.01, 0.02, ... 0.99
    y_vec = np.zeros(len(x_vec))
    line1 = []
    first_loop = True

    true_a = 0.05
    true_b = 0.03
    

    while not rospy.is_shutdown():
        #rospy.sleep(0.1)

        if my_listener.height_map is not None: #and my_listener.height_map_plane is not None:
            
            
            #print('looping')
            # plt.clf()

            #set plane to 99 and unobserved area to -99
            # height_map_combined = np.where(np.isnan(my_listener.height_map), -99, my_listener.height_map)
            # height_map_combined = np.where((height_map_combined==-99) & (np.logical_not(np.isnan(my_listener.height_map_plane))), 99, height_map_combined)

            # # try:
            # vmin=np.nanmin(my_listener.height_map)
            # vmax=np.nanmax(my_listener.height_map)
            # # except RuntimeWarning: 	
            # # 	print('runtimewarning')
            # # 	continue
            

            # cax = plt.imshow(height_map_combined, cmap=cmap, vmin = vmin, vmax=vmax)
            # fig.colorbar(cax)



            # np.save("/home/yusuke/height_map.npy", my_listener.height_map)
            # # np.save("/home/yusuke/height_map_plane.npy", my_listener.height_map_plane)
            # print('saved npy file')
            

            height_map_plotter(fig, ax1, my_listener.height_map, cmap, first_loop)

        
            contour, ellipse_list = plot_ellipse(my_listener.height_map)
            
           
            ax2.imshow(contour)
            ax2.set_title('contour map')
            
            if ellipse_list != []:
                (cx, cy), (d1, d2), angle = ellipse_list[0]
                # cx, cy = np.array(ellipse_list[0][0], dtype=int)
                # ellipse_axis =  ellipse_list[0][1]
                # a = max(ellipse_axis)
                # b = min(ellipse_axis)
                # angle = ellipse_list[0][2]
                
                a = max(d1, d2) / 2 * my_listener.resolution 
                b = min(d1, d2) / 2 * my_listener.resolution 

                y_vec[-1] = a
            
            line1 = size_plotter(fig, ax3, x_vec,y_vec,line1,true_value=true_a) #updates graph
            y_vec = np.append(y_vec[1:],0.0)   #append any value to keep y length

            first_loop = False
            plt.pause(0.1)

            

        #print(my_listerner.get_height_w(-2.0,1.0,2))
        else:
            rospy.loginfo("Waiting for height map...")
            rospy.sleep(1.0)
        
    #rospy.spin()








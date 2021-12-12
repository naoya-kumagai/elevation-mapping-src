#!/usr/bin/env python3

from numpy.core.defchararray import lower
from numpy.lib.stride_tricks import _maybe_view_as_subclass
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
from scipy.optimize import curve_fit
from scipy.stats import norm
import random
#from collections import Counter

def height_map_plotter(fig, ax, height_map, resolution, cmap, first_loop, vmin, vmax):
    '''
    Plots the 2D height map
    '''
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

def plot_height_dist(fig, ax, height_map, first_loop, plane_height_mean, plane_height_std=0.005):
    '''
    Plots histogram of the height destribution
    '''
    if first_loop:
        pass

    ax.clear()
    ax.set_title('Elevation distribution')
    ax.set_ylabel('number of cells')
    ax.set_xlabel('elevation (m)')
    #ax.set_xlim(-0.44, -0.41) #for plane only
    xmin, xmax = -0.44, -0.35
    ax.set_xlim(xmin,xmax)
    ax.hist(np.reshape(height_map, (-1,1)), bins = 100)
    ax.axvline(plane_height_mean, color='blue', lw=3, label='Plane height mean')
    ax.axvline(plane_height_mean+plane_height_std, color='green', lw=3, label='Plane height upper bound')
    x = np.linspace(xmin,xmax, 100)
    y = norm.pdf(x, plane_height_mean, plane_height_std)
    ax.plot(x,y, lw=3, label='Gaussian fit')
    ax.legend()

# def get_plane_height_mean(height_map):
#     '''
#     Returns the estimated plane height from the height map data
#     '''
#     min = -0.44
#     max = -0.34
#     hist, bin_edges = np.histogram(np.reshape(height_map, (-1,1)),bins=np.linspace(min,max, 100), density=False)
#     plane_height_mean = bin_edges[np.argmax(hist)]
#     return plane_height_mean

def get_plane_height_mean2(height_map, init_guess=(-0.43, 0.005, 100)):
    '''
    Fit the height distribution to a Gaussian using scipy.optimize
    By providing good initial guesses, the plane height can be predicted with high accuracy.
    '''
    # https://stackoverflow.com/questions/35990467/fit-mixture-of-two-gaussian-normal-distributions-to-a-histogram-from-one-set-of

    #change shape and exclude nan values 
    height_map_data = np.reshape(height_map, (-1,1))
    height_map_data = height_map_data[~np.isnan(height_map_data)]

    # A simple Gaussian fit does not work, since the bouldering hold height skews the mean to the right
    #mean, std = norm.fit(height_map_data)

    def gauss(x,mu,sigma,A):
        return A*np.exp(-(x-mu)**2/2/sigma**2)

    # def bimodal(x,mu1,sigma1,A1,mu2,sigma2,A2):
    #     return gauss(x,mu1,sigma1,A1)+gauss(x,mu2,sigma2,A2)

    init_guess_mean = -0.43
    init_guess_std = 0.005
    # About 2% of the data is the plane height. 
    # This depends on terrain and the definition of x below.
    height_map_length, height_map_width = height_map.shape
    init_guess_dist = height_map_length*height_map_width*0.02
    #init_guess_dist = 100
    init_guess = (init_guess_mean, init_guess_std, init_guess_dist)
    xmin, xmax = -0.44, -0.35
    x = np.linspace(xmin,xmax, 100)
    y, _ = np.histogram(height_map_data, bins=x)
    x=(x[1:]+x[:-1])/2 # for len(x)==len(y)

    params,cov=curve_fit(gauss,x,y,p0=init_guess)
    sigma=np.sqrt(cov)
    mean = params[0]

    return mean


def detect_from_dist(fig, ax, height_map, plane_height_mean, resolution, cmap, first_loop, vmin, vmax):
    '''
    Creates masking of height map based on the distribution of the data and plots the results.
    This masking assumes that the plane is the dominant surface area, which is reasonable for the current climbing wall.
    '''
    variance = 0.005 # assumes variance of plane data is 5mm. TODO: improve this!

    plane_height_upper_bound = plane_height_mean + variance

    height_map_masked = np.where(height_map > plane_height_upper_bound, height_map, np.nan)

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
    
    return height_map_masked

def mask_to_ellipse(fig, ax, height_map_masked, resolution, plane_height_mean, first_loop):
    '''
    Fits ellipses to mask 
    Returns list of all detected ellipses in the map 
    '''

    height_map_masked_binary = np.where(np.isnan(height_map_masked), 0, 255).astype(np.uint8)
    
    contours, hierarchy = cv2.findContours(height_map_masked_binary, 1, 2)

    rospy.loginfo(f'Detected {len(contours)} contours')
    if first_loop:
        ax.grid()
        ax.set_title('Ellipse fitting')
        ax.set_xlabel('map width (m)')
        ax.set_ylabel('map length (m)')
    extent = [0, height_map_masked.shape[1]*resolution, 0, height_map_masked.shape[0]*resolution]

    has_ellipse = len(contours) > 0
    ellipse_list = []
    if has_ellipse:

        ellipse_result_img = cv2.cvtColor(height_map_masked_binary, cv2.COLOR_GRAY2RGB)
      
        for cnt_num, cnt in enumerate(contours): 
            #Method 1: Opencv's fitellipse function
          
            try: 
                ellipse = cv2.fitEllipse(cnt)
                (xc, yc), (a,b), theta = ellipse 
                print(ellipse)
            except:
                continue
            
            #Method 2: skimage's EllipseModel class (least squares fiting) 
            #This fits an ellipse with accurate size but weird orientation at times
            # https://scikit-image.org/docs/dev/api/skimage.measure.html#skimage.measure.EllipseModel
            # try: 
            #     # print(cnt)
            #     cnt = np.array(cnt)
            #     cnt = np.squeeze(cnt)
            #     # print(np.array(cnt).shape)
            #     ellipse = EllipseModel()
            #     est_res = ellipse.estimate(cnt)
            #     # print(est_res)
            #     # print(ellipse.params)
            #     xc,yc,a,b,theta=ellipse.params
            
            # except:
            #     continue

            #Method 3: RANSAC TODO
          

            # def ransac(image, max_iter, threshold=5):
            #     ellipse_noise = image
            #     data = ellipse_noise
            #     ics = []
            #     best_ic = 0
            #     best_model = None
            #     xn, yn = data.nonzero()
            #     nzero = [(x1,y1) for x1, y1 in zip(xn, yn)]
            #     for epoch in range(max_iter):
            #         ic = 0
            #         sample = random.sample(nzero, 6)
            #         a = direct_least_square(np.array([s[0] for s in sample]), np.array([s[1] for s in sample]))
            #         for x, y in sample:
            #             eq = np.mat(np.vstack([x**2, x*y, y**2, x, y, 1])).T
            #             if np.abs(np.dot(eq, a.reshape(-1,1))) <= threshold:
            #                 ic += 1
            #         ics.append(ic)
            #         if ic > best_ic:
            #             best_ic = ic
            #             best_model = a
            #     return a, ics




            # If ellipse is too small or too big, ignore it
            lower_thres = 0.05/ resolution 
            upper_thres = 0.2 / resolution

            if (a < lower_thres or a > upper_thres) and (b < lower_thres or b> upper_thres):
                print('Eliminated contour that is too small')
                continue
            
            # https://stackoverflow.com/questions/23830618/python-opencv-typeerror-layout-of-the-output-array-incompatible-with-cvmat
            cimg = np.zeros_like(height_map_masked_binary, dtype=np.uint8).copy()

            cv2.drawContours(cimg, contours, cnt_num, 255, -1)
            height_inside_ctr = np.where(cimg==255, height_map_masked, np.NINF)
            # ax.imshow(height_inside_ctr)
            print(np.max(height_inside_ctr))
            if np.max(height_inside_ctr) < plane_height_mean + 0.02:
                continue
                      
                  
            try:
                center = (
                    int(round(xc)),
                    int(round(yc))
                )
                axes = (
                    int(round(a/2)),
                    int(round(b/2))
                )
            except ValueError: 
                continue


            ellipse_list.append(ellipse)
                      
            cv2.ellipse(ellipse_result_img, center, axes, theta, 0, 360, 255, 1)

            #draw_ellipse(img=ellipse_result_img, center=(xc, yc), axes=(a,b), angle=theta, color=(255,0,0))

        

        rospy.loginfo(f'Detected {len(ellipse_list)} ellipses')


        ax.imshow(ellipse_result_img, extent=extent)
     
    else:
        rospy.loginfo("no ellipse detected!")



def online_plotting(save_map):
    plt.style.use('ggplot')

    my_listener = listener() #the height map would be a top-down view

    fig, (ax1, ax2,ax3, ax4) = plt.subplots(nrows=1, ncols=4, figsize=(15,15), constrained_layout=True)
 
    cmap = plt.cm.rainbow
    cmap.set_under('black') # set ubobserved area to -99
    cmap.set_over('white')     # set planar region to 99
    cmap.set_bad('gray')


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
            vmax = vmin + 0.20
            

            # vmin = None
            # vmax = None

            height_map_plotter(fig, ax1, my_listener.height_map, my_listener.resolution, cmap, first_loop, vmin, vmax)
            plane_height_mean = get_plane_height_mean2(my_listener.height_map)
            plot_height_dist(fig, ax2, my_listener.height_map, first_loop, plane_height_mean)
            height_map_masked = detect_from_dist(fig, ax3,my_listener.height_map, plane_height_mean, my_listener.resolution, cmap, first_loop, vmin, vmax)
            mask_to_ellipse(fig, ax4, height_map_masked, my_listener.resolution, plane_height_mean, first_loop)


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
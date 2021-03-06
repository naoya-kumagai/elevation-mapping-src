#! /usr/bin/env python

import elevation_map_package.elevation_map_sub as sub

import rospy
import numpy as np
import cv2
import sys
import os
# import time
import matplotlib.pyplot as plt
# from scipy.optimize import curve_fit
from scipy.stats import norm

import yaml
from datetime import datetime

from sklearn.linear_model import RANSACRegressor


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

def plot_height_dist(fig, ax, height_map, first_loop, plane_height_mean, range_min=-0.03, range_max=0.10, plane_height_std=0.005):
    '''
    Plots histogram of the height destribution
    '''
    if first_loop:
        pass

    ax.clear()
    ax.set_title('Elevation distribution')
    ax.set_ylabel('number of cells')
    ax.set_xlabel('elevation (m)')

    # xmin, xmax = -0.44, -0.35

    xmin = plane_height_mean + range_min 
    xmax = plane_height_mean + range_max
 
    try:
        ax.set_xlim(xmin,xmax)
    except:
        pass

    try:
        ax.hist(np.reshape(height_map, (-1,1)), bins = 100)
    except ValueError as e:
        print(e)
        return
    
    ax.axvline(plane_height_mean, color='blue', lw=2, label='Plane height mean')
    ax.axvline(plane_height_mean+plane_height_std, color='green', lw=2, label='Plane height upper bound')
    x = np.linspace(xmin,xmax, 100)
    y = norm.pdf(x, plane_height_mean, plane_height_std)
    ax.plot(x,y, lw=2, label='Gaussian fit')
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

# def get_plane_height_mean2(height_map, init_guess_mean, range_min=-0.03, range_max=0.10):
#     '''
#     Fit the height distribution to a Gaussian using scipy.optimize
#     By providing good initial guesses, the plane height can be predicted with high accuracy.
#     '''
#     # https://stackoverflow.com/questions/35990467/fit-mixture-of-two-gaussian-normal-distributions-to-a-histogram-from-one-set-of

#     #change shape and exclude nan values 
#     height_map_data = np.reshape(height_map, (-1,1))
#     height_map_data = height_map_data[~np.isnan(height_map_data)]

#     # A simple Gaussian fit does not work, since the bouldering hold height skews the mean to the right
#     #mean, std = norm.fit(height_map_data)

#     def gauss(x,mu,sigma,A):
#         return A*np.exp(-(x-mu)**2/2/sigma**2)

#     # def bimodal(x,mu1,sigma1,A1,mu2,sigma2,A2):
#     #     return gauss(x,mu1,sigma1,A1)+gauss(x,mu2,sigma2,A2)

#     # init_guess_mean = -0.43
#     init_guess_std = 0.005
#     # About 2% of the data is the plane height. 
#     # This depends on terrain and the definition of x below. 
#     # This does not seem to make a big effect on the height estimation.
#     height_map_length, height_map_width = height_map.shape
#     init_guess_dist = height_map_length*height_map_width*0.02
#     #init_guess_dist = 100
#     init_guess = (init_guess_mean, init_guess_std, init_guess_dist)
#     # xmin, xmax = -0.44, -0.35

#     #Changing these parameters will make the plane estimation more tolerant to the initial guess error
#     xmin = init_guess_mean + range_min 
#     xmax = init_guess_mean + range_max

#     x = np.linspace(xmin,xmax, 100)
#     y, _ = np.histogram(height_map_data, bins=x)
#     x=(x[1:]+x[:-1])/2 # for len(x)==len(y)

#     try: 
#         params,cov=curve_fit(gauss,x,y,p0=init_guess)
#         #sigma=np.sqrt(cov)
#         mean = params[0]
#     except RuntimeError:
#         mean = init_guess_mean

   
#     # print(f'Mean predicted by Ransac: {mean}')
    
#     return mean

#import pyransac3d

def get_plane_height_mean3(height_map, resolution, init_guess_mean,ransac):
    '''
    Use the RANSAC algorithm to estimate the plane
    Robust to outliers compared to least squared fitting    
    '''

    extent = [0, height_map.shape[1]*resolution, 0, height_map.shape[0]*resolution]
    height_map_shape = height_map.shape
    height_map_ylen, height_map_xlen = height_map_shape

    xgrid, ygrid = np.meshgrid(np.linspace(0, height_map_xlen*resolution, height_map_xlen), 
                        np.linspace(0, height_map_ylen*resolution, height_map_ylen), indexing='xy')

    assert(xgrid.shape == height_map_shape)

    xyz = np.stack((xgrid, ygrid, height_map), axis=2)
    print(xyz.shape)


    
    #change shape and exclude nan values 
    height_map_data = np.reshape(height_map, (-1,1))
    # height_map_data = height_map_data[~np.isnan(height_map_data)]
    
    # try: 
    #     X = np.zeros([len(height_map_data),1])
    
    #     ransac.fit(, height_map_data)
    #     mean = ransac.predict([[0,]])

    #     inlier_mask = ransac.inlier_mask_
    #     print(height_map_shape)
    #     print(inlier_mask.shape)
    #     inlier_mask = np.reshape(inlier_mask, height_map_shape)
    #     print('Inlier mask')
    #     print(inlier_mask)

    #     converge_flag = True

    # except ValueError as e:
    #     print(e)
    #     mean = init_guess_mean

    #     converge_flag = False

    ########################################################

    # try: 
    #     X = np.zeros([len(height_map_data),1])
    
    #     ransac.fit(X, height_map_data)
    #     mean = ransac.predict([[0,]])

    #     inlier_mask = ransac.inlier_mask_
    #     print(height_map_shape)
    #     print(inlier_mask.shape)
    #     inlier_mask = np.reshape(inlier_mask, height_map_shape)
    #     print('Inlier mask')
    #     print(inlier_mask)

    #     converge_flag = True

    # except ValueError as e:
    #     print(e)
    #     mean = init_guess_mean

    #     converge_flag = False

    # return mean
    return 0


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

def detect_from_edges(fig,axes,  height_map, plane_height_mean, resolution, cmap, first_loop, vmin, vmax):
    extent = [0, height_map.shape[1]*resolution, 0, height_map.shape[0]*resolution]

    ax1, ax2 = axes


    # height_map_binary = np.where(np.isnan(height_map), 0, 255).astype(np.uint8)
    # print('Height map')
    # print(height_map)

    # def create_gray_cm(height_map):
    #     '''
        
    #     '''
    #     min_height = np.ma.masked_invalid(height_map).min()
    #     max_height = np.ma.masked_invalid(height_map).max()
    #     range = max_height - min_height
    #     print(range)
    #     scale = 255 / range
    #     print((np.where(np.ma.masked_invalid(height_map), 1, 0).min()))
    #     gray_cm = np.where(np.ma.masked_invalid(height_map), (height_map - min_height)*scale, 0.0)
    #     gray_cm = gray_cm.astype(int)
    #     # print(gray_cm)

    #     print(gray_cm.shape)
    #     print(gray_cm.min())
    #     print(gray_cm.max())

    #     return gray_cm

    
    # gray_cm = create_gray_cm(height_map)
    # gray_cm = np.zeros_like(height_map, dtype=np.uint8).copy()
    # height_map *= -1

    #Set values that are too low or high to the plane height
    height_map = np.where(height_map > plane_height_mean+0.15, plane_height_mean, height_map)
    height_map = np.where(height_map < plane_height_mean-0.05, plane_height_mean, height_map)

    #set the unobserved regions to the plane height
    height_map = np.where(np.isnan(height_map), plane_height_mean, height_map)

    #get min and max values
    min_height = np.min(height_map)
    max_height = np.max(height_map)
    # print(min_height, max_height)
    range = max_height - min_height
    # print(max_height - min_height)

    height_map_normalized = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    # print(np.min(height_map_normalized))
    # print(np.max(height_map_normalized))

    plane_height_normalized = int((plane_height_mean - min_height) / range * 255)
    plane_height_ub_normalized = int((plane_height_mean+0.05 - min_height) / range * 255)
    # print(plane_height_normalized)
    # print(type(plane_height_normalized))
   #  height_map_normalized = np.where(height_map_normalized==0, 0, height_map_normalized)


    blurred = cv2.GaussianBlur(height_map_normalized, (3,3), 0)
    print(blurred)

    #pixels below lower threshold are discarded
    #pixels between lower and higher threshold are considered only if they are connected to pixels in upper thres

    # edge = cv2.Canny(blurred, )

    # Utilize Canny Edge Detector
    edged = cv2.Canny(blurred, plane_height_normalized, plane_height_ub_normalized)

    # wide = auto_canny(height_map_normalized, 0.35)
    # tight = auto_canny(height_map_normalized, 0.95)
    

    if first_loop:
        ax1.grid()
        ax2.grid()

    # ax.imshow(height_map_normalized)


    ax1.imshow(blurred)
    ax2.imshow(edged)
        


# def auto_canny(image, sigma = 0.35):
#     # compute the mediam of the single channel pixel intensities
#     v = np.median(image)

#     # apply automatic Canny edge detection using the computed median
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) *v))
#     edged = cv2.Canny(image, lower, upper)

#     # return edged image
#     return edged

    
    

def mask_to_ellipse(fig, ax, height_map_masked, resolution, plane_height_mean, first_loop, verbose=False):
    '''
    Fits ellipses to mask 
    Returns list of all detected ellipses in the map 
    '''

    height_map_masked_binary = np.where(np.isnan(height_map_masked), 0, 255).astype(np.uint8)
    
    contours, hierarchy = cv2.findContours(height_map_masked_binary, 1, 2)

    if verbose:
        rospy.loginfo(f'Detected {len(contours)} contours')

    if first_loop:
        ax.grid()
        ax.set_title('Ellipse fitting')
        ax.set_xlabel('map width (m)')
        ax.set_ylabel('map length (m)')
    extent = [0, height_map_masked.shape[1]*resolution, 0, height_map_masked.shape[0]*resolution]

    has_ellipse = len(contours) > 0
    ellipse_list = []
    ellipse_dict = {}
    ellipse_num = 1
    if has_ellipse:

        ellipse_result_img = cv2.cvtColor(height_map_masked_binary, cv2.COLOR_GRAY2RGB)
      
        for cnt_num, cnt in enumerate(contours): 
            #Method 1: Opencv's fitellipse function
          
            try: 
                ellipse = cv2.fitEllipse(cnt)
                (xc, yc), (a,b), theta = ellipse 
              
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
            
            # Some heuristics for eliminating falsely detected objects
            
            # If ellipse is too small or too big, ignore it
            lower_thres = 0.05/ resolution #smaller than 0.05m = 5cm 
            upper_thres = 0.2 / resolution

            if (a < lower_thres or a > upper_thres) and (b < lower_thres or b> upper_thres):
                
                # if verbose:
                #     rospy.loginfo('Eliminated contour that is too small')
                continue
            
            # https://stackoverflow.com/questions/23830618/python-opencv-typeerror-layout-of-the-output-array-incompatible-with-cvmat
            cimg = np.zeros_like(height_map_masked_binary, dtype=np.uint8).copy()

            cv2.drawContours(cimg, contours, cnt_num, 255, -1)
            height_inside_ctr = np.where(cimg==255, height_map_masked, np.NINF)
            # ax.imshow(height_inside_ctr)
            
            # if np.nanmax(height_inside_ctr) < plane_height_mean + 0.02:
            #     continue

            if verbose:
                print(f'height inside ctr: {np.ma.masked_invalid(height_inside_ctr).mean()}' )
            

            if np.ma.masked_invalid(height_inside_ctr).mean() < plane_height_mean + 0.01:
                continue
                      
            # print(height_map_masked.shape[1])
            # print(height_map_masked.shape[0])

            # print(resolution)

            #Converts coordinates to actual size in meters and relative to the robot's 
            # initial position (middle of the grid map)
            # xc = (xc - height_map_masked.shape[1]/2)*resolution
            # yc = (height_map_masked.shape[0]/2 - yc)*resolution

            xc_actual = xc*resolution
            yc_actual = yc*resolution

       
            # print(xc, yc)

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

            print(np.ma.masked_invalid(height_inside_ctr).max())
            print(plane_height_mean[0])

            ellipsoid_height = (np.ma.masked_invalid(height_inside_ctr).max() - plane_height_mean[0])
            ellipsoid_height = float(ellipsoid_height)

          
            
            ellipse = {'Center_coordinates(cm)': [xc_actual*100, yc_actual*100], 'Axis_lengths(cm)': [a*resolution*100, b*resolution*100], 
            'Theta(degrees)': theta, 'Height(cm)': ellipsoid_height*100}

            ellipse_dict[ellipse_num] = ellipse
            ellipse_num += 1           
            #ellipse_list.append(ellipse)
                      
            cv2.ellipse(ellipse_result_img, center, axes, theta, 0, 360, 255, 1)

            #draw_ellipse(img=ellipse_result_img, center=(xc, yc), axes=(a,b), angle=theta, color=(255,0,0))

        
        if verbose:
            rospy.loginfo(f'Detected {ellipse_num - 1} ellipses')


        ax.imshow(ellipse_result_img, extent=extent)
     
    else:
        rospy.loginfo("no ellipse detected!")

    if verbose:
        print(ellipse_dict)
        
    return ellipse_dict

import tf
from geometry_msgs.msg import Point, Pose, Quaternion
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool

import pyransac3d
from scipy.spatial.transform import Rotation as R
import std_srvs
import tf_conversions


def get_plane_equation(plane_model, height_map, RESOLUTION, verbose=False):
    xgrid, ygrid = np.meshgrid(np.linspace(0, height_map.shape[1]*RESOLUTION, height_map.shape[1]), np.linspace(0, height_map.shape[0]*RESOLUTION, height_map.shape[0]))
    xyz = np.zeros((np.size(xgrid), 3))
    xyz[:,0] = np.reshape(xgrid, -1)
    xyz[:,1] = np.reshape(ygrid, -1)
    xyz[:,2] = np.reshape(height_map, -1)
    # plane = pyransac3d.Plane()
    best_eq, best_inliers = plane_model.fit(xyz, 0.01)
    if best_eq:
        best_eq = np.array(best_eq)
        if best_eq[3] < 0:
            best_eq *= -1
    else:    
        rospy.loginfo('Plane estimation not successful. Returning NAN coeffs')
        return None

    a,b,c,d = best_eq

    if verbose: 
        print('Got plane estimate')
        print(f'Equation of plane: {a:.2f} x + {b:.2f} y + {c:.2f} z + {d:.2f} = 0')

    # rmat = get_rotation_mat([a,b,c])
    # xyz_rotated = np.dot(xyz, rmat.T)
    # assert(xyz.shape==xyz_rotated.shape)
    # [a,b,c,d], best_inliers = plane_model.fit(xyz_rotated, 0.01)
    # if verbose:
    #     print(f'Equation of transformed plane: {a:.2f} x + {b:.2f} y + {c:.2f} z + {d:.2f} = 0')
    return np.array([a,b,c,d])

def get_rotation_mat(M):
    # https://stackoverflow.com/questions/9423621/3d-rotations-of-a-plane
    N = (0,0,1)
    c = np.dot(M,N)
    x,y,z = np.cross(M,N) / np.linalg.norm(np.cross(M,N))
    s = np.sqrt(1 - c*c)
    C = 1 - c
    rmat = np.array([[ x*x*C+c,    x*y*C-z*s,  x*z*C+y*s ],
                    [ y*x*C+z*s,  y*y*C+c,    y*z*C-x*s ],
                    [ z*x*C-y*s,  z*y*C+x*s,  z*z*C+c   ]])
    return rmat



def online_plotting(plane_height_init_guess=None, save_map=False):
    
    odom_pub = rospy.Publisher("plane_tf_info", Odometry, queue_size=10)
    plane_flag_pub = rospy.Publisher('plane_converge_info', Bool, queue_size=1)

    my_listener = sub.elevation_map_listener() #the height map would be a top-down view

    plt.style.use('ggplot')
    fig, ((ax1,ax2,ax3), (ax4, ax5, ax6))  = plt.subplots(nrows=2, ncols=3, figsize=(15,15), constrained_layout=True)
 
    cmap = plt.cm.rainbow
    cmap.set_under('black') # set ubobserved area to -99
    cmap.set_over('white')     # set planar region to 99
    cmap.set_bad('gray')


    # size = 100
    # x_vec = np.linspace(0,1,size+1)[0:-1] #0.0, 0.01, 0.02, ... 0.99
    # y_vec = np.zeros(len(x_vec))
    # line1 = []
    first_loop = True

    i = 0
    np.set_printoptions(formatter={'float': lambda x: "{0:0.5f}".format(x)})

    # ransac_regressor = RANSACRegressor()
    plane_height_mean = None

    import time
    timestr = time.strftime("%Y%m%d-%H%M%S")
    height_maps = []        

    coeffs_stack_len = 10
    coeffs_stack = np.zeros((coeffs_stack_len, 4))
    plane_converge_flag = False
    plane_model = pyransac3d.Plane()

    while not rospy.is_shutdown():

        # odom = Odometry()
        # odom.pose.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        # odom_pub.publish(odom)

        if my_listener.height_map is not None: #and my_listener.height_map_plane is not None:
        #if my_listener.uncertainty_range_map is not None:               

            # cax = plt.imshow(height_map_combined, cmap=cmap, vmin = vmin, vmax=vmax)
            # fig.colorbar(cax)
            # if first_loop:
            #     #initialize accumulated height map with 0s
            #     my_listener.accumulated_height_map = np.zeros(my_listener.height_map.shape)


            #if uncertainty is less than threshold, update that part of the accumulated height map 

            #my_listener.accumulated_height_map = np.where(my_listener.uncertainty_range_map < 0.01, my_listener.height_map, my_listener.accumulated_height_map)

            #dont change!!!!
            # vmin = -0.415 - 0.05
            # vmax = vmin + 0.20

            if plane_height_init_guess is None:
                try: 
                    plane_height_init_guess = np.nanmean(my_listener.height_map)
                except:
                    plane_height_init_guess = 0 


            #parameters for clear visualization of height map 
            range_min = -0.05
            range_max = 0.10


            if plane_height_mean is None: 
                vmin = plane_height_init_guess + range_min
                vmax = plane_height_init_guess + range_max
            
            else:
                vmin = plane_height_mean + range_min
                vmax = plane_height_mean + range_max

        
            # height_maps.append(my_listener.height_map)
            # import pickle
            # with open(f'map-{timestr}.pkl','wb') as f:
            #     pickle.dump(height_maps, f)



            # print(my_listener.height_map[0][0])
            

            coeffs = get_plane_equation(plane_model, my_listener.height_map, my_listener.resolution)
            if coeffs is None:
                i-=1
                continue
            coeffs_stack[i%coeffs_stack_len] = coeffs
            print(coeffs_stack[:,0])
            ax5.clear()
            # ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,0], label='a')
            # ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,1], label='b')
            # ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,2], label='c')
            # ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,3], label='d')

            ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,0], label='a')
            ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,1], label='b')
            ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,2], label='c')
            ax5.plot([j for j in range(coeffs_stack_len)], coeffs_stack[:,3], label='d')

            ax5.legend()


    
            coeffs_var = np.var(coeffs_stack, axis=0)
            print('Variance of coefficients')
            print(coeffs_var)
            if i>coeffs_stack_len and np.all(coeffs_var < 0.001):
                plane_converge_flag = True
            
            # if plane_converge_flag:
            # print(f'Plane estimation is stable')
            coeffs_mean = np.mean(coeffs_stack, axis=0)
            rmat = get_rotation_mat(coeffs_mean[:-1])
            pos_z = coeffs_mean[-1]

            q = R.from_matrix(rmat).as_quat()

            print(pos_z)
            print(q)

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose = Pose(Point(0,0,pos_z), Quaternion(*q))

            odom_pub.publish(odom)

            plane_flag_pub.publish(plane_converge_flag)
            
            #Change reference frame to plane-down view. Consequently, clear the map. 
            # my_listener.height_map.fill(np.nan)
            # rospy.ServiceProxy('/elevation_mapping/clear_map', std_srvs.srv.Empty())
            # print('successfully cleared map')


            # else:
            #     print(f'Not stable')

        
            


            height_map_plotter(fig, ax1, my_listener.height_map, my_listener.resolution, cmap, first_loop, vmin, vmax)
            
          
            # plane_height_mean = get_plane_height_mean3(my_listener.height_map,my_listener.resolution, plane_height_init_guess, ransac=ransac_regressor)

            # print(f'Plane height estimate: {plane_height_mean}')
            
            # plot_height_dist(fig, ax2, my_listener.height_map, first_loop, plane_height_mean, range_min=range_min, range_max=range_max)

            # height_map_gray = detect_from_edges(fig, (ax3, ax4), my_listener.height_map, plane_height_mean, my_listener.resolution, cmap, first_loop, vmin, vmax)




            # height_map_masked = detect_from_dist(fig, ax3,my_listener.height_map, plane_height_mean, my_listener.resolution, cmap, first_loop, vmin, vmax)
            # ellipse_dict = mask_to_ellipse(fig, ax4, height_map_masked, my_listener.resolution, plane_height_mean, first_loop, verbose=True)


            t = rospy.Time.now()
            file_path = os.path.dirname(__file__)
            path = os.path.join(file_path, "..", "..", "yaml_dir", f"{t}.yaml")
         
        
           
            # with open(path, 'w') as f:
            #     #print(ellipse_dict)
            #     yaml.safe_dump(ellipse_dict, f)
            #     print(f'Wrote to {t}.yaml')



            first_loop = False
            i += 1
            plt.pause(0.1) #same as plt.show(), kind of
            #plt.clf()
           

        else:
            rospy.loginfo("Waiting for height map...")
            rospy.sleep(1.0)    
            



if __name__ == '__main__':
    # print(os.path.dirname(sys.executable))
    # print(sys.version)
    # print(tf.__version__)
    plane_height_init_guess= -0.415
    # plane_height_init_guess= -0.33
    online_plotting(plane_height_init_guess=None, save_map=False)






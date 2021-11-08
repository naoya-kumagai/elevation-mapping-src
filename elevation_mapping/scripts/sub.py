#!/home/yusuke/miniconda3/envs/mask-rcnn/bin/python3.7


from IPython.utils.dir2 import dir2
from numpy.lib.shape_base import get_array_wrap
from grid_map_msgs.msg import GridMap
import rospy
from std_msgs.msg import String
import numpy as np

import cv2
import sys
import os
# ROOT_DIR = '/home/yusuke/Mask_RCNN'
# sys.path.append(ROOT_DIR)
# from mrcnn import utils
# from mrcnn import visualize
# from mrcnn.visualize import display_images
# import mrcnn.model as modellib
# from mrcnn.model import log
# sys.path.append(os.path.join(ROOT_DIR, "hold"))
# # sys.path.append(os.path.join(ROOT_DIR, "mrcnn"))
# # from mrcnn.inference import inference_init
# import tensorflow as tf
# import hold

import time
# from skimage.measure import find_contours
import matplotlib.pyplot as plt
from matplotlib import patches,  lines
from matplotlib.patches import Polygon


# import warnings


# def display_instances_realtime(image, boxes, masks, class_ids, class_names,
#                       scores=None, title="",
#                       figsize=(16, 16), ax=None,
#                       show_mask=True, show_bbox=True,
#                       colors=None, captions=None):
#     """
#     boxes: [num_instance, (y1, x1, y2, x2, class_id)] in image coordinates.
#     masks: [height, width, num_instances]
#     class_ids: [num_instances]
#     class_names: list of class names of the dataset
#     scores: (optional) confidence scores for each box
#     title: (optional) Figure title
#     show_mask, show_bbox: To show masks and bounding boxes or not
#     figsize: (optional) the size of the image
#     colors: (optional) An array or colors to use with each object
#     captions: (optional) A list of strings to use as captions for each object
#     """
#     # Number of instances
#     N = boxes.shape[0]
#     if not N:
#         print("\n*** No instances to display *** \n")
#     else:
#         assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]

#      # Generate random colors
#     colors = colors or visualize.random_colors(N)

#     # Show area outside image boundaries.
#     # height, width = image.shape[:2]
#     # ax.set_ylim(height + 10, -10)
#     # ax.set_xlim(-10, width + 10)
#     ax.axis('off')
#     ax.set_title(title)

#     masked_image = image.astype(np.uint32).copy()
#     for i in range(N):
#         color = colors[i]

#         # Bounding box
#         if not np.any(boxes[i]):
#             # Skip this instance. Has no bbox. Likely lost in image cropping.
#             continue
#         y1, x1, y2, x2 = boxes[i]
#         if show_bbox:
#             p = patches.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2,
#                                 alpha=0.7, linestyle="dashed",
#                                 edgecolor=color, facecolor='none')
#             ax.add_patch(p)

#         # Label
#         if not captions:
#             class_id = class_ids[i]
#             score = scores[i] if scores is not None else None
#             label = class_names[class_id]
#             caption = "{} {:.3f}".format(label, score) if score else label
#         else:
#             caption = captions[i]
#         ax.text(x1, y1 + 8, caption,
#                 color='w', size=11, backgroundcolor="none")

#         # Mask
#         mask = masks[:, :, i]
#         if show_mask:
#             masked_image =visualize.apply_mask(masked_image, mask, color)

#         # Mask Polygon
#         # Pad to ensure proper polygons for masks that touch image edges.
#         padded_mask = np.zeros(
#             (mask.shape[0] + 2, mask.shape[1] + 2), dtype=np.uint8)
#         padded_mask[1:-1, 1:-1] = mask
#         contours = find_contours(padded_mask, 0.5)
#         for verts in contours:
#             # Subtract the padding and flip (y, x) to (x, y)
#             verts = np.fliplr(verts) - 1
#             p = Polygon(verts, facecolor="none", edgecolor=color)
#             ax.add_patch(p)

#     ax.imshow(masked_image.astype(np.uint8))
#     print('refreshed image')

# def inference_init(ROOT_DIR, HOLD_DIR, subset, weights_path, DEVICE = "/gpu:0" ):

#     MODEL_DIR = os.path.join(ROOT_DIR, "logs")
#     # MODEL_WEIGHTS_PATH = ROOT_DIR +"/hold_mask_rcnn_coco.h5"

#     config = hold.CustomConfig()
#     # HOLD_DIR = ROOT_DIR+"/hold/newdata1"

#     # Override the training configurations with a few
#     # changes for inferencing.
#     class InferenceConfig(config.__class__):
#         # Run detection on one image at a time
#         GPU_COUNT = 1
#         IMAGES_PER_GPU = 1
#         # IMAGE_RESIZE_MODE = 'none'
#         # IMAGE_MIN_DIM = 256
#         # IMAGE_MAX_DIM = 1024

#     config = InferenceConfig()
#     config.display()

#     # set target device
#     # DEVICE = "/gpu:0"  # /cpu:0 or /gpu:0

#     dataset = hold.CustomDataset()
#     # dataset.load_custom(HOLD_DIR, "val")
#     dataset.load_custom(HOLD_DIR, subset)
#     # Must call before using the dataset
#     dataset.prepare()

#     print("Images: {}\nClasses: {}".format(len(dataset.image_ids), dataset.class_names))

#     # Create model in inference mode
#     with tf.device(DEVICE):
#         model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR,config=config)

#     #weights_path = "../logs/hold20211021T0846/mask_rcnn_hold_0010.h5"
#     # weights_path = "/content/drive/MyDrive/ColabNotebooks/logs/hold20211021T0846/mask_rcnn_hold_0010.h5"

#     # Load weights
#     print("Loading weights ", weights_path)
#     model.load_weights(weights_path, by_name=True)

#     return model, dataset, config

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
        self.height_map_filtered = None
        # self.height_map_plane = None
        self.uncertainty_range_map = None
        self.accumulated_height_map = None

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
        idx_uncertainty = gmdata.layers.index('uncertainty_range')
        

        self.robot_pose = gmdata.info.pose
        self.resolution = gmdata.info.resolution
        self.length_x = gmdata.info.length_x
        self.length_y = gmdata.info.length_y
        d = gmdata.data
        #print(len(d))
        self.height_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_elevation])
        self.uncertainty_range_map = translate_matrix(gmdata.outer_start_index,gmdata.inner_start_index,d[idx_uncertainty])
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

def edge_detect(ax, height_map):

    gray= cv2.normalize(src=height_map, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

    otsu, _ = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    edges = cv2.Canny(gray,otsu, otsu * 2, L2gradient = True)
        #edges = cv2.Canny(image=mask_gray, threshold1=100, threshold2=200)
    #print (otsu)
    contours, _ = cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # Applies convex hulls to each contour, ensuring each contour
    # is a closed polygon.
    hulls = map(cv2.convexHull,contours)




    
    # hulls_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    # print(hulls_img.shape)
    # cv2.drawContours(hulls_img,hulls,-1,(255,255,255),-1)

    ax.imshow(edges)
    return contours, edges 

def ellipse_fit(ax, contours, edges):
    # Draws contours onto a blank canvas
    mask = np.zeros(edges.shape,np.uint8)
    m3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    print(len(contours))
    for cnt in contours: 
        try: 
            ellipse = cv2.fitEllipse(cnt)
        except Exception as e:
            print(e)
            continue
        cv2.ellipse(m3, ellipse, color=(0,0,255), thickness=1)

    return m3



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

def height_map_plotter(fig, ax, height_map, cmap, first_loop, vmin, vmax):
    if first_loop == True:
        pos = ax.imshow(height_map,cmap=cmap, vmin =vmin, vmax =vmax)
        cbar = fig.colorbar(pos, ax=ax)
        #ax.set_ylabel('Y Label')
        ax.set_title('Height map')
    
    else:
        ax.imshow(height_map,cmap=cmap, vmin =vmin, vmax =vmax)

from mpl_toolkits.axes_grid1 import make_axes_locatable

def online_plotting(save_map):
    plt.style.use('ggplot')
    # warnings.filterwarnings("error")
    my_listener = listener() #the height map would be a top-down view
   
    fig = plt.figure()
    # fig, (ax1, ax2) = plt.subplots(figsize=(4,8),nrows=1, ncols=2)
    # model, dataset, config = inference_init(ROOT_DIR=ROOT_DIR, HOLD_DIR='/home/yusuke/Mask_RCNN/hold/dataset2', 
    #     subset='val',weights_path='/home/yusuke/Mask_RCNN/logs/mask_rcnn_hold_0010.h5')

    # fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
    # ax1.grid()
    # ax2.grid()
    # ax3.grid()
    # fig.tight_layout() 
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

        #if my_listener.height_map is not None: #and my_listener.height_map_plane is not None:
        if my_listener.uncertainty_range_map is not None:               

            # cax = plt.imshow(height_map_combined, cmap=cmap, vmin = vmin, vmax=vmax)
            # fig.colorbar(cax)
            if first_loop:
                #initialize accumulated height map with 0s
                my_listener.accumulated_height_map = np.zeros(my_listener.height_map.shape)


            if save_map:
                # np.save("/home/yusuke/height_map.npy", my_listener.height_map)
                # np.save("/home/yusuke/height_map_filtered.npy", my_listener.height_map_filtered)
                # # # np.save("/home/yusuke/height_map_plane.npy", my_listener.height_map_plane)
                # # 
                # print('saved npy file')
                gray= cv2.normalize(src=my_listener.height_map, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
              
                gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                
                hold_num = 'SEllipsoid'
                dataset_num = '_test3'
                angle = ''
                # cv2.imwrite(f'/home/yusuke/Mask_RCNN/hold/dataset{dataset_num}/hold_{hold_num}_{angle}{i}.png', gray)
                cv2.imwrite(f'/home/yusuke/Mask_RCNN/hold/evaluation_2.5/midsize_height50cm_dist100cm_{i}.png', gray)
                print('saved image')

            #if uncertainty is less than threshold, update that part of the accumulated height map 

            my_listener.accumulated_height_map = np.where(my_listener.uncertainty_range_map < 0.02, my_listener.height_map, my_listener.accumulated_height_map)

            vmin = -0.415 - 0.02
            vmax = vmin + 0.12

            #plt.imshow(my_listener.height_map, cmap=cmap, vmin =vmin, vmax =vmax)
            plt.imshow(my_listener.accumulated_height_map, cmap=cmap, vmin=vmin, vmax=vmax)
            plt.colorbar()
            # print(my_listener.uncertainty_range_map)

           




            # height_map_plotter(fig, ax1, my_listener.height_map_filtered, cmap, first_loop, vmin, vmax)
            # mask = np.zeros(my_listener.height_map.shape)
            masked = np.ma.masked_where(((vmin < my_listener.height_map) & (my_listener.height_map < vmax)), my_listener.height_map)
            
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
            plt.clf()
           

        #print(my_listerner.get_height_w(-2.0,1.0,2))
        else:
            rospy.loginfo("Waiting for height map...")
            rospy.sleep(1.0)    
            

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

if __name__ == '__main__':
    # print(os.path.dirname(sys.executable))
    # print(sys.version)
    # print(tf.__version__)
    online_plotting(False)
    #offline_test()




#!/usr/bin/env python3

from elevation_map_sub import listener
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

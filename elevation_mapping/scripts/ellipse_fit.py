#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
# from PIL import Image, ImageFilter
import cv2
#import seaborn as sns

height_map = np.load("/home/yusuke/height_map.npy")

mask = np.where(np.isnan(height_map), 0, 255).astype(np.uint8)
ret, thresh = cv2.threshold(mask, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, 1, 2)
m3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
has_ellipse = len(contours) > 0

if has_ellipse:
    for cnt in contours: 

        ellipse = cv2.fitEllipse(cnt)

        cx, cy = np.array(ellipse[0], dtype=int)
        a, b = ellipse[1]
        angle = ellipse[2]
        print(a,b)
        #plot center
        # m3[cy-2:cy+2,cx-2:cx+2] = (255, 0, 0)
        cv2.ellipse(m3, ellipse, (0,0,255), 1)
else:
    print('no ellipse detected!')


# cv2.imshow('contours',m3)
# cv2.waitKey(0)



plt.imshow(m3)
plt.show()











# fig = plt.figure(figsize=(4,8))
# cax = plt.matshow(height_map, fignum=0)
# fig.colorbar(cax)
# plt.show()

# image = Image.fromarray(height_map)

# image = image.filter()

# image = vis2 = cv2.cvtColor(height_map, cv2.COLOR_GRAY2BGR)
# print(np.nanmax(height_map))
# image = height_map.astype(np.uint8)
# print(np.max(image))
# print(np.where(image!=0))
# image = np.zeros(height_map.shape)

# image = cv2.normalize(src=height_map, dst=height_map, alpha=0, beta=255)

# cv2.imshow("Height Map", image)
# cv2.waitKey(0)
# print(image)
# print(np.nanmax(height_map))


# edges = cv2.Canny(height_map, 0, 1.0)
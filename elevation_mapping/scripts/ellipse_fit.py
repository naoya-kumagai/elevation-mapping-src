#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageFilter
import cv2


height_map = np.load("/home/yusuke/height_map.npy")

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


edges = cv2.Canny(height_map, 0, 1.0)
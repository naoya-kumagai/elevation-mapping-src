from os import access
import numpy as np
import matplotlib.pyplot as plt

hm = np.load(f'/home/yusuke/Mask_RCNN/hold/evaluation3/hold7_dist50cm_{20}.npy')
hm_data = np.reshape(hm, (-1,1))
print(hm_data.shape)
# plt.grid()

fig, (ax1, ax2) = plt.subplots(1,2)

ax1.hist(hm_data, bins = 30)
pos = ax2.imshow(hm, cmap='rainbow')
fig.colorbar(pos, ax=ax2)
plt.show()
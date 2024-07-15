import numpy as np
translation = np.array([3,3,1])
xmin, xmax = -5, 5
ymin, ymax = -5, 5
zmin, zmax = -1, 1
translation[0] = np.clip(translation[0], xmin, xmax)
translation[1] = np.clip(translation[1], ymin, ymax)
translation[2] = np.clip(translation[2], zmin, zmax)
print(translation)
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle
import glob
import cv2
from utils import *

method = "our"
scenario = 5
counter = 4
if scenario == 1:
    from CreateModel1 import *
elif scenario == 2:
    from CreateModel2 import *
elif scenario == 3:
    from CreateModel3 import *
elif scenario == 4:
    from CreateModel4 import *
elif scenario == 5:
    from CreateModel5 import *

with open("data/scen{}_{}_{}.txt".format(scenario, method, counter), "rb") as f:
    d  = pickle.load(f)
    paths = d["paths"]
# print(paths)

fig = plt.figure()
ax = fig.add_subplot(111)

# Plot Model
if OBS_RECTANGLE is not None:
    for (ox, oy, w, h) in OBS_RECTANGLE:
        ax.add_patch(patches.Rectangle((ox, oy), w, h,
                                        edgecolor='black',
                                        facecolor='none',
                                        fill=True))

if OBS_CIRCLE is not None:
    for (ox, oy, r) in OBS_CIRCLE:
        ax.add_patch(patches.Circle((ox, oy), r,
                                    edgecolor='black',
                                    facecolor='none',
                                    fill=True))

# Plot path
for path in paths:
    ax.plot(np.array(path)[:,0], np.array(path)[:,1], "blue", linewidth=2)

# Plot start and goals
plt.plot(START[0], START[1], "bs", linewidth=5)
plt.plot(np.array(GOALS)[:,0], np.array(GOALS)[:,1], "rs", linewidth=5)

image = cv2.imread('experiment.jpeg')
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.axis("scaled")
ax.set_xlim(X_RANGE)
ax.set_ylim(Y_RANGE)
plt.tight_layout()

plt.show()
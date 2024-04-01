import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle
import glob

from utils import *

scenario = 1
methods = ["rrt", "jianyou", "our"]
colors = ["green", "red", "blue"]
names = ["Theta-RRT", "FN-RRT", "Our"]

if scenario == 1:
    from CreateModel1 import *

## Plot paths
fig = plt.figure()
ax = fig.add_subplot(111)

# Plot Model
if OBS_RECTANGLE is not None:
    for (ox, oy, w, h) in OBS_RECTANGLE:
        ax.add_patch(patches.Rectangle((ox, oy), w, h,
                                        edgecolor='black',
                                        facecolor='black',
                                        fill=True))

if OBS_CIRCLE is not None:
    for (ox, oy, r) in OBS_CIRCLE:
        ax.add_patch(patches.Circle((ox, oy), r,
                                    edgecolor='black',
                                    facecolor='black',
                                    fill=True))

# Plot path
for i, method in enumerate(methods):
    print(method)
    file = open(glob.glob("data/scen{}_{}_*.txt".format(scenario, method))[0], "rb")
    d = pickle.load(file)
    paths = d["paths"]
    for path in paths:
        ax.plot(np.array(path)[:,0], np.array(path)[:,1], colors[i], linewidth=2)
    ax.plot([], [], colors[i], label=names[i])

# Plot start and goals
plt.plot(START[0], START[1], "bs", linewidth=5)
plt.plot(np.array(GOALS)[:,0], np.array(GOALS)[:,1], "rs", linewidth=5)

ax.legend()
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.axis("scaled")
ax.set_xlim(X_RANGE)
ax.set_ylim(Y_RANGE)
plt.tight_layout()
plt.show()

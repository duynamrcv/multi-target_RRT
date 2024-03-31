import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle
import glob

from utils import *

method = "rrt"
scenario = 1
if scenario == 1:
    from CreateModel1 import *

with open(glob.glob("data/scen{}_{}_*.txt".format(scenario, method))[0], "rb") as f:
    paths = pickle.load(f)
# print(paths)

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
for path in paths:
    ax.plot(np.array(path)[:,0], np.array(path)[:,1], "green", linewidth=2)

# Plot start and goals
plt.plot(START[0], START[1], "bs", linewidth=5)
plt.plot(np.array(GOALS)[:,0], np.array(GOALS)[:,1], "rs", linewidth=5)

ax.set_xlim(X_RANGE)
ax.set_ylim(Y_RANGE)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.axis("scaled")

plt.show()
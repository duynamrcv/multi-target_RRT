import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle
import glob

from utils import *

scenario = 1
methods = ["fmt", "rrt_smart", "our"]
colors = ["green", "red", "blue"]
names = ["FMT", "RRT*-Smart", "Our"]

plt.figure(figsize=(3,4))
data = []
for i, method in enumerate(methods):
    files = glob.glob("data/scen{}_{}_*.txt".format(scenario, method))
    pts = []
    for file in files:
        d = pickle.load(open(file, "rb"))
        pt = d["pt"]
        pts.append(pt)
        print("Method: {} - Computing time = {:.2f}".format(method, pt))
    pts = np.array(pts)
    data.append(pts)

medianprops = dict(linewidth=1.5, color='firebrick')
boxprops = dict(linewidth=1.5, color="black")
plt.boxplot(data, widths=0.2, whis=(0,100), sym="",
            medianprops=medianprops, boxprops=boxprops,
            labels=[method for method in names])
plt.ylabel("Computational time [s]")
plt.tight_layout()
plt.show()
    
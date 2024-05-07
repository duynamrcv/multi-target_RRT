import time
import math
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
file_name = "reduce"

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

data = np.array(data)
val_max = data.max(1)
val_min = data.min(1)
val_mean = data.mean(1)
val_std = data.std(1)

plt.errorbar(names, val_mean, [val_mean - val_min, val_max - val_mean],
            capsize=3, fmt="b--X", ecolor = "black",)
plt.ylabel("Computational time [s]")
plt.tight_layout()
plt.grid(axis='y')
plt.savefig("result/{}_scen{}_time.pdf".format(file_name, scenario), format="pdf", bbox_inches="tight")
plt.show()
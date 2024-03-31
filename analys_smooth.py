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

def compute_smooth(path):
    smooth = 0
    for i in range(len(path)-1):
        smooth += math.hypot(path[i][0]-path[i+1][0],
                             path[i][1]-path[i+1][1])
    return smooth

plt.figure()
data = []
for i, method in enumerate(methods):
    files = glob.glob("data/scen{}_{}_*.txt".format(scenario, method))
    smooths = []
    for file in files:
        d = pickle.load(open(file, "rb"))
        paths = d["paths"]
        smooth = 0
        for path in paths:
            smooth += compute_smooth(path)
        smooth /=  len(paths)
        smooths.append(smooth)
        # print("Method: {} - average smooth = {:.2f}".format(method, smooth))
    smooths = np.array(smooths)
    data.append(smooths)

medianprops = dict(linewidth=2.0, color='firebrick')
plt.boxplot(data, widths=0.2, whis=2.0, sym="", medianprops=medianprops,
            labels=[method for method in methods])
plt.ylabel("smooth [m]")
plt.tight_layout()
plt.show()
    
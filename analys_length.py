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

def compute_length(path):
    length = 0
    for i in range(len(path)-1):
        length += math.hypot(path[i][0]-path[i+1][0],
                             path[i][1]-path[i+1][1])
    return length

plt.figure()
data = []
for i, method in enumerate(methods):
    files = glob.glob("data/scen{}_{}_*.txt".format(scenario, method))
    lengths = []
    for file in files:
        d = pickle.load(open(file, "rb"))
        paths = d["paths"]
        length = 0
        for path in paths:
            length += compute_length(path)
        length /=  len(paths)
        lengths.append(length)
        # print("Method: {} - average length = {:.2f}".format(method, length))
    lengths = np.array(lengths)
    data.append(lengths)

medianprops = dict(linewidth=2.0, color='firebrick')
plt.boxplot(data, widths=0.2, whis=2.0, sym="", medianprops=medianprops,
            labels=[method for method in methods])
plt.ylabel("Length [m]")
plt.tight_layout()
plt.show()
    

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
file_name = "reduce"

def compute_length(path):
    length = 0
    for i in range(len(path)-1):
        length += math.hypot(path[i][0]-path[i+1][0],
                             path[i][1]-path[i+1][1])
    return length

plt.figure(figsize=(3,4))
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

medianprops = dict(linewidth=1.5, color='firebrick')
boxprops = dict(linewidth=1.5, color="black")
plt.boxplot(data, widths=0.2, whis=(0,100), sym="",
            medianprops=medianprops, boxprops=boxprops,
            labels=[method for method in names])
plt.ylabel("Length [m]")
plt.tight_layout()
plt.savefig("result/{}_scen{}_length.pdf".format(file_name, scenario), format="pdf", bbox_inches="tight")
plt.show()
    

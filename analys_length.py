import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle
import glob
from scipy import stats

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

fig = plt.figure(figsize=(3,3))
ax = fig.subplots()
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

data = np.array(data)
val_max = data.max(1)
val_min = data.min(1)
val_mean = data.mean(1)
val_std = data.std(1)

q1 = val_mean + val_std*stats.norm.ppf(0.3)
q3 = val_mean + val_std*stats.norm.ppf(0.7)
whislo = q1 - (q3 - q1)*1.5
whishi = q3 + (q3 - q1)*1.5

keys = ['med', 'q1', 'q3', 'whislo', 'whishi']
stats = [dict(zip(keys, vals)) for vals in zip(val_mean, q1, q3, val_min, val_max)]
ax.set_xticklabels(names)
plt.subplot().bxp(stats, showfliers=False)
plt.ylabel("Length [m]")
plt.tight_layout()
plt.grid(axis='y')
plt.savefig("result/{}_scen{}_length.pdf".format(file_name, scenario), format="pdf", bbox_inches="tight")
plt.show()
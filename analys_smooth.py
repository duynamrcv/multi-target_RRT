import time
import math
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

def dot_product(vec1, vec2):
    return sum(x * y for x, y in zip(vec1, vec2))

def vector_magnitude(vec):
    return math.sqrt(sum(x**2 for x in vec))
    
def compute_smooth(path):
    smooth = 0
    count = 0
    for i in range(1,len(path)-1):
        vec1 = [path[i][0]-path[i-1][0],
                path[i][1]-path[i-1][1]]
        vec2 = [path[i+1][0]-path[i][0],
                path[i+1][1]-path[i][1]]
        dot_prod = dot_product(vec1, vec2)
        mag_vec1 = vector_magnitude(vec1)
        mag_vec2 = vector_magnitude(vec2)
        if mag_vec1 == 0 or mag_vec2 == 0:
            count += 1
        else:
            cos_angle = dot_prod / (mag_vec1 * mag_vec2)
            if abs(cos_angle) > 1:
                count += 1
            else:
                # print(dot_prod, mag_vec1, mag_vec2, cos_angle)
                gamma = math.acos(cos_angle)
                smooth += gamma
    return smooth/(len(path)-2-count)

fig = plt.figure(figsize=(3,3))
ax = fig.subplots()
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
        print("Method: {} - average smooth = {:.2f}".format(method, smooth))
    smooths = np.array(smooths)
    data.append(smooths)

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
plt.ylabel("Smooth value")
plt.tight_layout()
plt.grid(axis='y')
plt.savefig("result/{}_scen{}_smooth.pdf".format(file_name, scenario), format="pdf", bbox_inches="tight")
plt.show()
    
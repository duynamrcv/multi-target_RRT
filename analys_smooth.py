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
            # print(dot_prod, mag_vec1, mag_vec2, cos_angle)
            gamma = math.acos(cos_angle)
            smooth += gamma
    return smooth/(len(path)-2-count)

plt.figure(figsize=(3,4))
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

medianprops = dict(linewidth=2.0, color='firebrick')
plt.boxplot(data, widths=0.2, whis=2.0, sym="", medianprops=medianprops,
            labels=[method for method in methods])
plt.ylabel("smooth [m]")
plt.tight_layout()
plt.show()
    
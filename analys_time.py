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
# methods = ["star", "smart", "fmt","our"]
# colors = ["cyan", "purple","orange", "blue"]
# names = ["RRT*", "RRT*-Smart","FMT", "Our"]
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

medianprops = dict(linewidth=1.5, color='firebrick')
boxprops = dict(linewidth=1.5, color="black")
bp = plt.boxplot(data, widths=0.2, whis=(0,100), sym="",patch_artist=True,labels=[method for method in names])
for box in bp['boxes']:
    box.set(color='black', linewidth=1.5) 
    box.set(facecolor='yellow')  
plt.ylabel("Computational time [s]")
plt.tight_layout()
plt.savefig("result/{}_scen{}_time.pdf".format(file_name, scenario), format="pdf", bbox_inches="tight")
plt.show()
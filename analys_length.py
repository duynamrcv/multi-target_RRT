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

for i, method in enumerate(methods):
    files = glob.glob("data/scen{}_{}_*.txt".format(scenario, method))
    for file in files:
        paths = pickle.load(open(file, "rb"))
        length = 0
        for path in paths:
            length += compute_length(path)
        length /=  len(paths)
    print("Method: {} - average length = {:.2f}".format(method, length))
    

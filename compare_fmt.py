import os
import sys
import math
import numpy as np
import random

from utils import *

# select model
scenario = 1
counter = 0
if scenario == 1:
    from CreateModel1 import *
elif scenario == 2:
    from CreateModel2 import *
elif scenario == 3:
    from CreateModel3 import *
elif scenario == 4:
    from CreateModel4 import *
class FMT:
    def __init__(self, x_start, x_goals):
        self.x_init = Node(x_start)
        self.x_goals = [Node(x_goal) for x_goal in x_goals]

        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        self.sample_numbers = 1000

    def Init(self):
        samples = self.SampleFree()

        self.x_init.cost = 0.0
        self.V.add(self.x_init)
        self.V.update(samples)
        self.V_unvisited.update(samples)
        for x_goal in self.x_goals:
            self.V_unvisited.add(x_goal)
        self.V_open.add(self.x_init)

    def planning(self):
        self.Init()
        z = self.x_init
        n = self.sample_numbers
        rn = 10*SEARCH_RADIUS * math.sqrt((math.log(n) / n))
        Visited = []
        count = 0
        iter = 0

        # while z is not self.x_goals:
        while count != len(self.x_goals) or iter < MAX_ITER:
            V_open_new = set()
            X_near = self.Near(self.V_unvisited, z, rn)
            Visited.append(z)

            for x in X_near:
                Y_near = self.Near(self.V_open, x, rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_near}
                y_min = min(cost_list, key=cost_list.get)

                if not is_collision(y_min, x, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)

            if not self.V_open:
                print("open set empty!")
                break

            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)
            for x_goal in self.x_goals:
                if z is x_goal:
                    count += 1 
            iter += 1
        
        paths = []
        for x_goal in self.x_goals:
            path_x, path_y = self.ExtractPath(x_goal)
            path = np.array([path_x, path_y]).T
            paths.append(path)
        return 1, paths

    def ExtractPath(self, x_goal):
        path_x, path_y = [], []
        node = x_goal

        while node.parent:
            path_x.append(node.x)
            path_y.append(node.y)
            node = node.parent

        path_x.append(self.x_init.x)
        path_y.append(self.x_init.y)

        return path_x, path_y

    def Cost(self, x_start, x_end):
        if is_collision(x_start, x_end, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
            return np.inf
        else:
            return self.calc_dist(x_start, x_end)

    @staticmethod
    def calc_dist(x_start, x_end):
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)

    @staticmethod
    def Near(nodelist, z, rn):
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def SampleFree(self):
        n = self.sample_numbers
        Sample = set()

        ind = 0
        while ind < n:
            node = Node((random.uniform(X_RANGE[0] + ROBOT_RADIUS, X_RANGE[1] - ROBOT_RADIUS),
                         random.uniform(Y_RANGE[0] + ROBOT_RADIUS, Y_RANGE[1] - ROBOT_RADIUS)))
            if is_inside_obs(node, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample
        
import pickle
import time

if __name__ == "__main__":
    plan = FMT(START, GOALS)
    
    st = time.time()
    # path_x, path_y = plan.planning()
    success, paths = plan.planning()
    # vertex = rrt.vertex
    print(paths)
    pt = time.time() - st
    if success:
        print("FMT done: {:.4f}s".format(pt))
        with open('data/scen{}_fmt_{}.txt'.format(scenario, counter), 'wb') as f:
            d = dict()
            d["paths"] = paths
            d["pt"] = pt
            pickle.dump(d, f)
    else:
        print("Failed.")
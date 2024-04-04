import os
import sys
import math
import numpy as np

from utils import *

# select model
scenario = 1
counter = 0
if scenario == 1:
    from CreateModel1 import *
elif scenario == 2:
    from CreateModel2 import *

import math
import random
import matplotlib.pyplot as plt
import numpy as np
import scipy
import time

class RrtStarSmart:
    def __init__(self, x_start, x_goals):
        self.x_start = Node(x_start)
        self.x_goals = [Node(x_goal) for x_goal in x_goals]
        
        self.V = [self.x_start]
        self.beacons = [[] for i in range(len(x_goals))]
        self.beacons_radius = 2
        self.direct_cost_old = np.inf
        self.obs_vertex = get_obs_vertex(OBS_RECTANGLE, ROBOT_RADIUS)

    def planning(self):
        n = 0
        b = 2
        InitPathFlag = False
        self.ReformObsVertex()

        for k in range(MAX_ITER):
            if k % 200 == 0:
                print(k)

            for i in range(len(self.x_goals)):
                if (k - n) % b == 0 and len(self.beacons[i]) > 0:
                    x_rand = self.Sample(self.beacons[i])
                else:
                    x_rand = self.Sample()

                x_nearest = self.Nearest(self.V, x_rand)
                x_new = self.Steer(x_nearest, x_rand)

                if x_new and not is_collision(x_nearest, x_new, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                    X_near = self.Near(self.V, x_new)
                    self.V.append(x_new)

                    if X_near:
                        # choose parent
                        cost_list = [self.Cost(x_near) + self.Line(x_near, x_new) for x_near in X_near]
                        x_new.parent = X_near[int(np.argmin(cost_list))]

                        # rewire
                        c_min = self.Cost(x_new)
                        for x_near in X_near:
                            c_near = self.Cost(x_near)
                            c_new = c_min + self.Line(x_new, x_near)
                            if c_new < c_near:
                                x_near.parent = x_new

                    if not InitPathFlag and self.InitialPathFound(x_new, self.x_goals[i]):
                        InitPathFlag = True
                        n = k

                    if InitPathFlag:
                        self.PathOptimization(x_new, i)
                    # if k % 5 == 0:
                    #     self.animation()

        paths = []
        for x_goal in self.x_goals:
            path = self.ExtractPath(x_goal)
            paths.append(path)
        return 1, paths

    def PathOptimization(self, node, i):
        direct_cost_new = 0.0
        node_end = self.x_goals[i]

        while node.parent:
            node_parent = node.parent
            if not is_collision(node_parent, node_end, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                node_end.parent = node_parent
            else:
                direct_cost_new += self.Line(node, node_end)
                node_end = node

            node = node_parent

        if direct_cost_new < self.direct_cost_old:
            self.direct_cost_old = direct_cost_new
            self.UpdateBeacons(i)

    def UpdateBeacons(self, i):
        node = self.x_goals[i]
        beacons = []

        while node.parent:
            near_vertex = [v for v in self.obs_vertex
                           if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9]
            if len(near_vertex) > 0:
                for v in near_vertex:
                    beacons.append(v)

            node = node.parent

        self.beacons[i] = beacons

    def ReformObsVertex(self):
        obs_vertex = []

        for obs in self.obs_vertex:
            for vertex in obs:
                obs_vertex.append(vertex)

        self.obs_vertex = obs_vertex

    def Steer(self, x_start, x_goal):
        dist, theta = self.get_distance_and_angle(x_start, x_goal)
        dist = min(STEP_LENGTH, dist)
        node_new = Node((x_start.x + dist * math.cos(theta),
                         x_start.y + dist * math.sin(theta)))
        node_new.parent = x_start

        return node_new

    def Near(self, nodelist, node):
        n = len(self.V) + 1
        r = 50 * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2 and
                  not is_collision(node, nodelist[ind], OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)]

        return X_near

    def Sample(self, goal=None):
        if goal is None:

            if np.random.random() > GOAL_SAMPLE_RATE:
                return Node((np.random.uniform(X_RANGE[0] + ROBOT_RADIUS, X_RANGE[1] - ROBOT_RADIUS),
                             np.random.uniform(Y_RANGE[0] + ROBOT_RADIUS, Y_RANGE[1] - ROBOT_RADIUS)))

            return self.x_goals[np.random.randint(len(self.x_goals))]
        else:
            R = self.beacons_radius
            r = random.uniform(0, R)
            theta = random.uniform(0, 2 * math.pi)
            ind = random.randint(0, len(goal) - 1)

            return Node((goal[ind][0] + r * math.cos(theta),
                         goal[ind][1] + r * math.sin(theta)))

    def SampleFreeSpace(self):
        if np.random.random() > GOAL_SAMPLE_RATE:
            return Node((np.random.uniform(X_RANGE[0] + ROBOT_RADIUS, X_RANGE[1] - ROBOT_RADIUS),
                             np.random.uniform(Y_RANGE[0] + ROBOT_RADIUS, Y_RANGE[1] - ROBOT_RADIUS)))

        return self.x_goals[np.random.randint(len(self.x_goals))]

    def ExtractPath(self, node):
        path = []

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.x_start.x, self.x_start.y])

        return path

    def InitialPathFound(self, node, x_goal):
        if self.Line(node, x_goal) < STEP_LENGTH:
            return True

        return False

    @staticmethod
    def Nearest(nodelist, n):
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2
                                       for nd in nodelist]))]

    @staticmethod
    def Line(x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    @staticmethod
    def Cost(node):
        cost = 0.0
        if node.parent is None:
            return cost

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    
import pickle
import time

if __name__ == "__main__":
    rrt = RrtStarSmart(START, GOALS)
    
    st = time.time()
    success, paths = rrt.planning()
    # vertex = rrt.vertex
    print(paths)
    pt = time.time() - st
    if success:
        print("RRT done: {:.4f}s".format(pt))
        with open('data/scen{}_rrt_smart_{}.txt'.format(scenario, counter), 'wb') as f:
            d = dict()
            d["paths"] = paths
            d["pt"] = pt
            pickle.dump(d, f)
    else:
        print("Failed.")

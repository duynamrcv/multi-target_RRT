import os
import sys
import math
import random
import numpy as np

from utils import *

# select model
scenario = 1
counter = 0
if scenario == 1:
    from CreateModel1 import *
elif scenario == 2:
    from CreateModel2 import *

class Tree:
    def __init__(self, x_start, x_goals):
        self.x_start = x_start
        self.goals = [x_goal for x_goal in x_goals]

        self.r = 10*SEARCH_RADIUS
        self.V = set()
        self.E = set()
        self.QE = set()
        self.QV = set()

        self.V_old = set()


class BITStar:
    def __init__(self, x_start, x_goals, eta=5):
        self.x_start = Node(x_start)
        self.x_goals = [Node(x_goal) for x_goal in x_goals]
        self.eta = eta

        self.Tree = Tree(self.x_start, self.x_goals)
        self.X_sample = set()
        self.g_T = dict()

    def init(self):
        self.Tree.V.add(self.x_start)
        for x_goal in self.x_goals:
            self.X_sample.add(x_goal)

        self.g_T[self.x_start] = 0.0
        for x_goal in self.x_goals:
            self.g_T[x_goal] = np.inf

        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        cx = self.x_start.x; cy = self.x_start.y
        for x_goal in self.x_goals:
            cx += x_goal.x; cy += x_goal.y
        xCenter = np.array([[cx / (len(self.x_goals)+1)],
                            [cy / (len(self.x_goals)+1)], [0.0]])

        return theta, cMin, xCenter, C

    def planning(self):
        theta, cMin, xCenter, C = self.init()

        for _ in range(MAX_ITER):
            if not self.Tree.QE and not self.Tree.QV:
                if _ == 0:
                    m = 350
                else:
                    m = 200

                count = 0
                for x_goal in self.x_goals:
                    if x_goal.parent is not None:
                        path_x, path_y = self.ExtractPath(x_goal)
                        count += 1
                if count == len(self.x_goals):
                    break

                for x_goal in self.x_goals:
                    self.Prune(self.g_T[x_goal])
                    self.X_sample.update(self.Sample(m, self.g_T[x_goal], cMin, xCenter, C))
                self.Tree.V_old = {v for v in self.Tree.V}
                self.Tree.QV = {v for v in self.Tree.V}

            while self.BestVertexQueueValue() <= self.BestEdgeQueueValue():
                self.ExpandVertex(self.BestInVertexQueue())

            vm, xm = self.BestInEdgeQueue()
            self.Tree.QE.remove((vm, xm))

            for x_goal in self.x_goals:
                if self.g_T[vm] + self.calc_dist(vm, xm) + self.h_estimated(xm) < self.g_T[x_goal]:
                    actual_cost = self.cost(vm, xm)
                    if self.g_estimated(vm) + actual_cost + self.h_estimated(xm) < self.g_T[x_goal]:
                        if self.g_T[vm] + actual_cost < self.g_T[xm]:
                            if xm in self.Tree.V:
                                # remove edges
                                edge_delete = set()
                                for v, x in self.Tree.E:
                                    if x == xm:
                                        edge_delete.add((v, x))

                                for edge in edge_delete:
                                    self.Tree.E.remove(edge)
                            else:
                                self.X_sample.remove(xm)
                                self.Tree.V.add(xm)
                                self.Tree.QV.add(xm)

                            self.g_T[xm] = self.g_T[vm] + actual_cost
                            self.Tree.E.add((vm, xm))
                            xm.parent = vm

                            set_delete = set()
                            for v, x in self.Tree.QE:
                                if x == xm and self.g_T[v] + self.calc_dist(v, xm) >= self.g_T[xm]:
                                    set_delete.add((v, x))

                            for edge in set_delete:
                                self.Tree.QE.remove(edge)
                # else:
                #     self.Tree.QE = set()
                #     self.Tree.QV = set()

            # if k % 5 == 0:
            #     self.animation(xCenter, self.g_T[self.x_goal], cMin, theta)

        paths = []
        for x_goal in self.x_goals:
            path_x, path_y = self.ExtractPath(x_goal)
            path = np.array([path_x, path_y]).T
            paths.append(path)
        return 1, paths
        
    def ExtractPath(self, node):
        path_x, path_y = [node.x], [node.y]

        while node.parent:
            node = node.parent
            path_x.append(node.x)
            path_y.append(node.y)

        return path_x, path_y

    def Prune(self, cBest):
        self.X_sample = {x for x in self.X_sample if self.f_estimated(x) < cBest}
        self.Tree.V = {v for v in self.Tree.V if self.f_estimated(v) <= cBest}
        self.Tree.E = {(v, w) for v, w in self.Tree.E
                       if self.f_estimated(v) <= cBest and self.f_estimated(w) <= cBest}
        self.X_sample.update({v for v in self.Tree.V if self.g_T[v] == np.inf})
        self.Tree.V = {v for v in self.Tree.V if self.g_T[v] < np.inf}

    def cost(self, start, end):
        if is_collision(start, end, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
            return np.inf

        return self.calc_dist(start, end)

    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)

    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node)

    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal)

    def Sample(self, m, cMax, cMin, xCenter, C):
        if cMax < np.inf:
            return self.SampleEllipsoid(m, cMax, cMin, xCenter, C)
        else:
            return self.SampleFreeSpace(m)

    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
        L = np.diag(r)

        ind = 0
        Sample = set()

        while ind < m:
            xBall = self.SampleUnitNBall()
            x_rand = np.dot(np.dot(C, L), xBall) + xCenter
            node = Node([x_rand[(0, 0)], x_rand[(1, 0)]])
            in_obs = is_inside_obs(node, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)
            in_x_range = X_RANGE[0] + ROBOT_RADIUS <= node.x <= X_RANGE[1] - ROBOT_RADIUS
            in_y_range = Y_RANGE[0] + ROBOT_RADIUS <= node.y <= Y_RANGE[1] - ROBOT_RADIUS

            if not in_obs and in_x_range and in_y_range:
                Sample.add(node)
                ind += 1

        return Sample

    def SampleFreeSpace(self, m):
        Sample = set()

        ind = 0
        while ind < m:
            node = Node([random.uniform(X_RANGE[0] + ROBOT_RADIUS, X_RANGE[1] - ROBOT_RADIUS),
                         random.uniform(Y_RANGE[0] + ROBOT_RADIUS, Y_RANGE[1] - ROBOT_RADIUS)])
            if is_inside_obs(node, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample

    def radius(self, q):
        cBest = self.g_T[self.x_goal]
        lambda_X = len([1 for v in self.Tree.V if self.f_estimated(v) <= cBest])
        radius = 2 * self.eta * (1.5 * lambda_X / math.pi * math.log(q) / q) ** 0.5

        return radius

    def ExpandVertex(self, v):
        self.Tree.QV.remove(v)
        X_near = {x for x in self.X_sample if self.calc_dist(x, v) <= self.Tree.r}

        for x in X_near:
            if self.g_estimated(v) + self.calc_dist(v, x) + self.h_estimated(x) < self.g_T[self.x_goal]:
                self.g_T[x] = np.inf
                self.Tree.QE.add((v, x))

        if v not in self.Tree.V_old:
            V_near = {w for w in self.Tree.V if self.calc_dist(w, v) <= self.Tree.r}

            for w in V_near:
                if (v, w) not in self.Tree.E and \
                        self.g_estimated(v) + self.calc_dist(v, w) + self.h_estimated(w) < self.g_T[self.x_goal] and \
                        self.g_T[v] + self.calc_dist(v, w) < self.g_T[w]:
                    self.Tree.QE.add((v, w))
                    if w not in self.g_T:
                        self.g_T[w] = np.inf

    def BestVertexQueueValue(self):
        if not self.Tree.QV:
            return np.inf

        return min(self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV)

    def BestEdgeQueueValue(self):
        if not self.Tree.QE:
            return np.inf

        return min(self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE)

    def BestInVertexQueue(self):
        if not self.Tree.QV:
            print("QV is Empty!")
            return None

        v_value = {v: self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV}

        return min(v_value, key=v_value.get)

    def BestInEdgeQueue(self):
        if not self.Tree.QE:
            print("QE is Empty!")
            return None

        e_value = {(v, x): self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE}

        return min(e_value, key=e_value.get)

    @staticmethod
    def SampleUnitNBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T

        return C

    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.x - end.x, start.y - end.y)

    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

import pickle
import time

if __name__ == "__main__":
    plan = BITStar(START, GOALS)
    
    st = time.time()
    success, paths = plan.planning()
    # vertex = rrt.vertex
    print(paths)
    pt = time.time() - st
    if success:
        print("RRT done: {:.4f}s".format(pt))
        with open('data_rrt/scen{}_bit_{}.txt'.format(scenario, counter), 'wb') as f:
            d = dict()
            d["paths"] = paths
            d["pt"] = pt
            pickle.dump(d, f)
    else:
        print("Failed.")

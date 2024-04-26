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
elif scenario == 3:
    from CreateModel3 import *
elif scenario == 4:
    from CreateModel4 import *
class RRTStar:
    def __init__(self, x_start:Node, x_goals:Node):
        self.s_start = Node(x_start)
        self.s_goals = [Node(x_goal) for x_goal in x_goals]
        self.vertex = [self.s_start]

    def planning(self):
        for _ in range(MAX_ITER):
            node_rand = self.generate_random_node(GOAL_SAMPLE_RATE)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not is_collision(node_near, node_new, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)

                if neighbor_index:
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

        paths = []
        for goal in self.s_goals:
            index = self.search_goal_parent(goal)
            path = self.extract_path(self.vertex[index], goal)
            paths.append(path)
        success = 1
        return success, paths

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(STEP_LENGTH, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def search_goal_parent(self, goal):
        dist_list = [math.hypot(n.x - goal.x, n.y - goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= STEP_LENGTH]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not is_collision(self.vertex[i], goal, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self, goal_sample_rate):
        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(X_RANGE[0] + ROBOT_RADIUS, X_RANGE[1] - ROBOT_RADIUS),
                         np.random.uniform(Y_RANGE[0] + ROBOT_RADIUS, Y_RANGE[1] - ROBOT_RADIUS)))

        return self.s_goals[np.random.randint(len(self.s_goals))]

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(SEARCH_RADIUS * math.sqrt((math.log(n) / n)), STEP_LENGTH)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not is_collision(node_new, self.vertex[ind], OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)]

        return dist_table_index

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def extract_path(self, node_end, goal):
        path = [[goal.x, goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    
import pickle
import time

if __name__ == "__main__":
    rrt = RRTStar(START, GOALS)
    
    st = time.time()
    success, paths = rrt.planning()
    # vertex = rrt.vertex
    print(paths)
    pt = time.time() - st
    if success:
        print("RRT done: {:.4f}s".format(pt))
        with open('data/scen{}_rrtstar_{}.txt'.format(scenario, counter), 'wb') as f:
            d = dict()
            d["paths"] = paths
            d["pt"] = pt
            pickle.dump(d, f)
    else:
        print("Failed.")
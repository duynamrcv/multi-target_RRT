import os
import sys
import math
import numpy as np

from utils import *

# select model
scenario = 1
if scenario == 1:
    from CreateModel1 import *
elif scenario == 2:
    from CreateModel2 import *

class RRT:
    def __init__(self, x_start:Node, x_goals:Node):
        self.s_start = Node(x_start)
        self.s_goal = [Node(goal) for goal in x_goals]

        self.vertex = [self.s_start]
        self.paths = []
        for _ in range(len(self.s_goal)):
            self.paths.append([])

    def planning(self):
        success = 0
        for _ in range(MAX_ITER):
            node_rand = self.generate_random_node()
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and angle_thresh(node_near, node_new, MAX_ANGLE) \
                    and not is_collision(node_near, node_new, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                self.vertex.append(node_new)

                for i in range(len(self.s_goal)):
                    dist, _ = self.get_distance_and_angle(node_new, self.s_goal[i])
                    if dist <= STEP_LENGTH and angle_thresh(node_near, node_new, MAX_ANGLE) \
                                            and not is_collision(node_new, self.s_goal[i], OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS):
                        index = self.search_goal_parent(i)
                        self.paths[i] = self.extract_path(self.vertex[index], i)
            
            # Are all paths done?
            check = 0
            for i in range(len(self.paths)):
                if len(self.paths[i]) != 0:
                    check += 1
            if check == len(self.s_goal):
                success = 1
                break
        print("[INFO] RRT planning done!.")
        return success, self.paths

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(STEP_LENGTH, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def search_goal_parent(self, idx):
        dist_list = [math.hypot(n.x - self.s_goal[idx].x, n.y - self.s_goal[idx].y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= STEP_LENGTH]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not is_collision(self.vertex[i], self.s_goal[idx], OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self):
        if np.random.random() > GOAL_SAMPLE_RATE:
            return Node((np.random.uniform(X_RANGE[0] + ROBOT_RADIUS, X_RANGE[1] - ROBOT_RADIUS),
                         np.random.uniform(Y_RANGE[0] + ROBOT_RADIUS, Y_RANGE[1] - ROBOT_RADIUS)))

        return self.s_goal[np.random.randint(len(self.s_goal))]

    def nearest_neighbor(self, node_list:Node, n):
        
        dis_node = [math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]

        return node_list[int(np.argmin(dis_node))]

    @staticmethod
    def cost(node_p:Node):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def extract_path(self, node_end, idx):
        path = [[self.s_goal[idx].x, self.s_goal[idx].y]]
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
    rrt = RRT(START, GOALS)
    
    st = time.time()
    success, paths = rrt.planning()
    # vertex = rrt.vertex
    print(paths)
    pt = time.time() - st
    if success:
        print("RRT done: {:.4f}s".format(pt))
        with open('data/scen{}_rrt_{:.4f}.txt'.format(scenario, pt), 'wb') as f:
            pickle.dump(paths, f)
    else:
        print("Failed.")

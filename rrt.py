import os
import sys
import math
import numpy as np

import env, utils
from utils import Node
from bezier import Bezier

class RRT:
    def __init__(self, x_start, x_goal, thangle, step_len,
                 goal_sample_rate, search_radius, iter_max):
        self.start = x_start
        self.goals = x_goal
        self.s_start = Node(x_start)
        self.s_goal = [Node(goal) for goal in x_goal]
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

        self.thangle = thangle

        self.vertex = [self.s_start]
        self.paths = []
        for _ in range(len(self.s_goal)):
            self.paths.append([])

        self.env = env.Env()
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def planning(self):
        for _ in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and self.utils.angle_thresh(node_near, node_new) \
                    and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)

                for i in range(len(self.s_goal)):
                    dist, _ = self.get_distance_and_angle(node_new, self.s_goal[i])
                    if dist <= self.step_len and self.utils.angle_thresh(node_near, node_new) \
                                            and not self.utils.is_collision(node_new, self.s_goal[i]):
                        index = self.search_goal_parent(i)
                        self.paths[i] = self.extract_path(self.vertex[index], i)
            
            # Are all paths done?
            check = 0
            for i in range(len(self.paths)):
                if len(self.paths[i]) != 0:
                    check += 1
            if check == len(self.s_goal):
                break
        for i in range(len(self.paths)):
            path = self.paths[i]
            path = self.remove_residual_node(path, self.start, self.goals[i])
            path = self.curve_path(path, self.start, self.goals[i])
            self.paths[i] = path
        return self.paths

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def search_goal_parent(self, idx):
        dist_list = [math.hypot(n.x - self.s_goal[idx].x, n.y - self.s_goal[idx].y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.is_collision(self.vertex[i], self.s_goal[idx])]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal[np.random.randint(len(self.s_goal))]

    def nearest_neighbor(self, node_list, n):
        
        dis_node = [math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]

        return node_list[int(np.argmin(dis_node))]

    @staticmethod
    def cost(node_p):
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

    def remove_residual_node(self, path, start, goal):
        new_path = [start]

        while new_path[-1] != goal:
            for i in range(len(path)):
                if not self.utils.is_collision(Node(path[i]), Node(new_path[-1])) \
                        and self.utils.angle_thresh( Node(new_path[-1]), Node(path[i])):
                    new_path.append(path[i])
                    break
        return new_path

    def curve_path(self, path, start, goal):
        new_path = [start]
        for i in range(1, len(path)-1):
            p = path[i]
            dis = self.utils.min_distance_to_obs(Node(p))

            p1 = path[i-1]
            p2 = path[i+1]

            ang1 = math.atan2(p1[1] - p[1], p1[0] - p[0])
            new1 = [p[0] + dis*math.cos(ang1), p[1] + dis*math.sin(ang1)]
            ang2 = math.atan2(p2[1] - p[1], p2[0] - p[0])
            new2 = [p[0] + dis*math.cos(ang2), p[1] + dis*math.sin(ang2)]

            bs = Bezier(new1, p, new2)
            out = bs.evaluate(10)
            new_path.extend(out)
            
        new_path.append(goal)
        return new_path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

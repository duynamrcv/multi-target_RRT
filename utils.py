import math
import numpy as np

import env
from config import *

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class Utils:
    def __init__(self):
        self.env = env.Env()

        self.delta = rr
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
    
    def angle_thresh(self, node:Node, node_new:Node):
        pnode = node.parent
        if pnode == None:
            return True
        
        vec1 = [node_new.x-node.x, node_new.y-node.y]
        vec2 = [node.x-pnode.x, node.y-pnode.y]

        x = math.hypot(vec1[0], vec1[1])*math.hypot(vec2[0], vec2[1])
        if x == 0:
            return True
        else:
            cos_ang = (vec1[0]*vec2[0] + vec1[1]*vec2[1])/x
            if abs(cos_ang) > 1:
                cos_ang = math.copysign(1, cos_ang)
            angle = math.acos(cos_ang)
            return angle <= thangle

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                if self.is_intersect_circle(o, d, [x, y], r):
                    return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                if math.hypot(node.x - x, node.y - y) <= r + delta:
                    return True

        if self.obs_rectangle is not None:
            for (x, y, w, h) in self.obs_rectangle:
                if 0 <= node.x - (x - delta) <= w + 2 * delta \
                        and 0 <= node.y - (y - delta) <= h + 2 * delta:
                    return True

        return False

    def min_distance_to_obs(self, node):
        diss = []

        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                dis = self.get_dist(node, Node((x, y))) - self.delta - r
                diss.append(dis)

        if self.obs_rectangle is not None:
            for (x, y, w, h) in self.obs_rectangle:
                if 0 <= node.x - (x - self.delta) < w + 2*self.delta:
                    dis = min(abs(node.y - y), abs(node.y - (y+h))) - self.delta
                elif 0 <= node.y - (y - self.delta) < h + 2*self.delta:
                    dis = min(abs(node.x - x), abs(node.x - (x+w))) - self.delta
                else:
                    d1 = self.get_dist(node, Node((x,y))) - self.delta
                    d2 = self.get_dist(node, Node((x+w,y))) - self.delta
                    d3 = self.get_dist(node, Node((x,y+h))) - self.delta
                    d4 = self.get_dist(node, Node((x+w,y+h))) - self.delta
                    dis = min(d1, d2, d3, d4)
                diss.append(dis)
        return min(diss)

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
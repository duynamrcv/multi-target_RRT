import math
import numpy as np

class Node:
    def __init__(self, n:list):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.cost = np.inf
    
def angle_thresh(node:Node, node_new:Node, max_angle):
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
        return angle <= max_angle

def get_obs_vertex(obs_rectangle:list, delta:float):
    obs_list = []

    for (ox, oy, w, h) in obs_rectangle:
        vertex_list = [[ox - delta, oy - delta],
                        [ox + w + delta, oy - delta],
                        [ox + w + delta, oy + h + delta],
                        [ox - delta, oy + h + delta]]
        obs_list.append(vertex_list)

    return obs_list

def is_intersect_rec(start, end, o, d, a, b):
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
        dist_obs = get_dist(start, shot)
        dist_seg = get_dist(start, end)
        if dist_obs <= dist_seg:
            return True

    return False

def is_intersect_circle(o, d, a, r, delta):
    d2 = np.dot(d, d)

    if d2 == 0:
        return False

    t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

    if 0 <= t <= 1:
        shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
        if get_dist(shot, Node(a)) <= r + delta:
            return True

    return False

def is_collision(start:Node, end:Node, obs_rectangle:list, obs_circle:list, delta:float):
    if is_inside_obs(start, obs_rectangle, obs_circle, delta) \
        or is_inside_obs(end, obs_rectangle, obs_circle, delta):
        return True

    o, d = get_ray(start, end)
    obs_vertex = get_obs_vertex(obs_rectangle, delta)

    for (v1, v2, v3, v4) in obs_vertex:
        if is_intersect_rec(start, end, o, d, v1, v2):
            return True
        if is_intersect_rec(start, end, o, d, v2, v3):
            return True
        if is_intersect_rec(start, end, o, d, v3, v4):
            return True
        if is_intersect_rec(start, end, o, d, v4, v1):
            return True

    if obs_circle is not None:
        for (x, y, r) in obs_circle:
            if is_intersect_circle(o, d, [x, y], r, delta):
                return True

    return False

def is_inside_obs(node:Node, obs_rectangle:list, obs_circle:list, delta:float):
    if obs_circle is not None:
        for (x, y, r) in obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

    if obs_rectangle is not None:
        for (x, y, w, h) in obs_rectangle:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

    return False

def min_distance_to_obs(node:Node, obs_rectangle:list, obs_circle:list, delta:float):
    diss = []

    if obs_circle is not None:
        for (x, y, r) in obs_circle:
            dis = get_dist(node, Node((x, y))) - delta - r
            diss.append(dis)

    if obs_rectangle is not None:
        for (x, y, w, h) in obs_rectangle:
            if 0 <= node.x - (x - delta) < w + 2*delta:
                dis = min(abs(node.y - y), abs(node.y - (y+h))) - delta
            elif 0 <= node.y - (y - delta) < h + 2*delta:
                dis = min(abs(node.x - x), abs(node.x - (x+w))) - delta
            else:
                d1 = get_dist(node, Node((x,y))) - delta
                d2 = get_dist(node, Node((x+w,y))) - delta
                d3 = get_dist(node, Node((x,y+h))) - delta
                d4 = get_dist(node, Node((x+w,y+h))) - delta
                dis = min(d1, d2, d3, d4)
            diss.append(dis)
    return min(diss)

def get_ray(start:Node, end:Node):
    orig = [start.x, start.y]
    direc = [end.x - start.x, end.y - start.y]
    return orig, direc

def get_dist(start:Node, end:Node):
    return math.hypot(end.x - start.x, end.y - start.y)
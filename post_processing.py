import time
import pickle
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import *
from bezier import Bezier

def remove_residual_node(path, start, goal, obs_rectangle, obs_circle, delta):
    new_path = [start]
    count = 500
    while new_path[-1] != goal and count > 0:
        for i in range(len(path)):
            if not is_collision(Node(new_path[-1]), Node(path[i]), obs_rectangle, obs_circle, delta):
                new_path.append(path[i])
                break
        count -= 1
    return count, new_path

def curve_path(path, obs_rectangle, obs_circle, delta):
    new_path = [path[0]]
    for i in range(1, len(path)-1):
        p = path[i]
        dis = ROBOT_RADIUS + min_distance_to_obs(Node(p), obs_rectangle, obs_circle, delta)

        p1 = path[i-1]
        p2 = path[i+1]

        # ang1 = math.atan2(p1[1] - p[1], p1[0] - p[0])
        # new1 = [p[0] + dis*math.cos(ang1), p[1] + dis*math.sin(ang1)]
        # ang2 = math.atan2(p2[1] - p[1], p2[0] - p[0])
        # new2 = [p[0] + dis*math.cos(ang2), p[1] + dis*math.sin(ang2)]

        new1 = [0.5*(p[0]+p1[0]), 0.5*(p[1]+p1[1])]
        new2 = [0.5*(p[0]+p2[0]), 0.5*(p[1]+p2[1])]

        bs = Bezier(new1, p, new2)
        out = bs.evaluate(5)
        new_path.extend(out)
        
    new_path.append(path[-1])
    return new_path

scenario = 1
if scenario == 1:
    from CreateModel1 import *
elif scenario == 2:
    from CreateModel2 import *

file = glob.glob("data/scen{}_rrt_*.txt".format(scenario))[0]
with open(file, "rb") as f:
    paths = pickle.load(f)

pt = float(file[:-4].split("_")[-1])

st = time.time()
new_paths = []
success = 0
for id in range(len(GOALS)):
    count, new_path = remove_residual_node(paths[id], START, GOALS[id], OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)
    if count == 0:
        print("Failed")
    else:
        new_path = curve_path(new_path, OBS_RECTANGLE, OBS_CIRCLE, ROBOT_RADIUS)
        new_paths.append(new_path)
        success += 1
pt += time.time() - st

if success == len(GOALS):
    with open('data/scen{}_our_{:.4f}.txt'.format(scenario, pt), 'wb') as f:
        pickle.dump(paths, f)
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # Plot Model
    if OBS_RECTANGLE is not None:
        for (ox, oy, w, h) in OBS_RECTANGLE:
            ax.add_patch(patches.Rectangle((ox, oy), w, h,
                                            edgecolor='black',
                                            facecolor='black',
                                            fill=True))

    if OBS_CIRCLE is not None:
        for (ox, oy, r) in OBS_CIRCLE:
            ax.add_patch(patches.Circle((ox, oy), r,
                                        edgecolor='black',
                                        facecolor='black',
                                        fill=True))

    # Plot path
    for path in paths:
        ax.plot(np.array(path)[:,0], np.array(path)[:,1], "green", linewidth=2)
    for new_path in new_paths:
        ax.plot(np.array(new_path)[:,0], np.array(new_path)[:,1], "blue", linewidth=2)

    # Plot start and goals
    plt.plot(START[0], START[1], "bs", linewidth=5)
    plt.plot(np.array(GOALS)[:,0], np.array(GOALS)[:,1], "rs", linewidth=5)

    ax.set_xlim(X_RANGE)
    ax.set_ylim(Y_RANGE)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("scaled")

    plt.show()
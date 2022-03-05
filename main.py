import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from bezier import Bezier
from rrt import RRT
from config import *
from utils import *

class Plotting:
    def __init__(self, start, goals, vertex):
        self.start = start
        self.goals = goals
        self.vertex = vertex

        self.env = env.Env()
        # self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.bg = cv2.imread('sydney1.jpg')
        # self.bg = cv2.imread('PIA24011.jpg')

    def plot_grid(self, name):
        fig, ax = plt.subplots()
        
        ax.imshow(cv2.cvtColor(self.bg, cv2.COLOR_BGR2RGB), origin='lower')

        if self.obs_rectangle is not None:
            for (ox, oy, w, h) in self.obs_rectangle:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='red',
                        facecolor='none',
                        linewidth='2',
                        fill=True
                    )
                )

        if self.obs_circle is not None:
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r,
                        edgecolor='red',
                        facecolor='none',
                        linewidth='2',
                        fill=True
                    )
                )
        if self.obs_circle is not None:
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r*3/4,
                        edgecolor='red',
                        facecolor='none',
                        linewidth='2',
                        fill=True
                    )
                )
        if self.obs_circle is not None:
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r/2,
                        edgecolor='red',
                        facecolor='none',
                        linewidth='2',
                        fill=True
                    )
                )
        if self.obs_circle is not None:
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), 5,
                        edgecolor='red',
                        facecolor='red',
                        fill=True
                    )
                )

        plt.plot(self.start[0], self.start[1], "bs", linewidth=5)
        plt.plot(np.array(self.goals)[:,0], np.array(self.goals)[:,1], "rs", linewidth=5)

        plt.title(name)
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')

    def plot_path(self, paths, c='green'):
        for path in paths:
            plt.plot(np.array(path)[:,0], np.array(path)[:,1], c, linewidth=2)
    
    def plot_visited(self):
        for node in self.vertex:
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

if __name__ == "__main__":
    # Init start position and goal positions
    start = [540,210] # Starting node
    goals =[[160,320],[580,710],[950,350]] # Goal node

    # Init algorithm
    rrt_star = RRT(start, goals, thangle, step_len, goal_sample_rate, search_radius, iter_max)
    
    st = time.time()
    st1=time.time()
    paths = rrt_star.planning()
    vertex = rrt_star.vertex
    print("RRT done: {:.2f}s".format(time.time()-st))

    plot = Plotting(start, goals, vertex)
    plot.plot_grid("Scenario 1")
    plot.plot_visited()
    plot.plot_path(paths, c='yellow')
    plt.show()
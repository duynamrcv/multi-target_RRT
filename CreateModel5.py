import math
import numpy as np

# Environment dimension
XDIM = 1000
YDIM = 800

X_RANGE = [0, XDIM]
Y_RANGE = [0, YDIM]

START = [128, 364]
GOALS = [[487, 140],
         [460,  580],
         [880, 540]]

# Obstacle
OBS_CIRCLE = [[600,  120, 50],
              [840,  715, 80],
              [360, 420, 55],
              [600,  330, 60],
              [900,  420, 50],
              [248, 240, 70]]

OBS_RECTANGLE = [[120,  520,  300, 280],
                 [765,  140,  235, 200],
                 [600, 0, 400,  130],
                 [253, 0, 347,  105],
                 [390, 170, 100,150],
                 [510, 440, 90,120],
                 [690, 510, 80, 130],
                 [110, 410, 130, 80]]

# Robot constraints
ROBOT_RADIUS = 20
MAX_ANGLE = np.deg2rad(77.922) #1.36 rad

# RRT configs
STEP_LENGTH = 50
GOAL_SAMPLE_RATE = 0.01
SEARCH_RADIUS = 80
MAX_ITER = 5000
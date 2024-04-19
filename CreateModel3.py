import math
import numpy as np

# Environment dimension
XDIM = 1000
YDIM = 800

X_RANGE = [0, XDIM]
Y_RANGE = [0, YDIM]

START = [600, 60]
GOALS = [[808, 100],
         [890, 750],
         [80, 700]]

# Obstacle
OBS_CIRCLE = [[120, 240, 40],
              [760, 480, 60],
              [560, 320, 40],
              [500, 710, 60],
              [270, 600, 40],
              [350, 100, 60],
              [870, 320, 40],
              [390, 360, 40]]
OBS_RECTANGLE = [[720, 0, 40, 200],
                 [480, 200, 340, 40],
                 [650, 600,  350,  40],
                 [100, 600, 40,  200],
                 [250, 300, 40,  160],
                 [100, 460, 400, 40],
                 [400, 500, 40, 100],
                 [635, 240, 40, 150],
                 [740, 640, 40, 60],
                 [180, 0, 40, 140]]

# Robot constraints
ROBOT_RADIUS = 10
MAX_ANGLE = np.deg2rad(60.0)

# RRT configs
STEP_LENGTH = 50
GOAL_SAMPLE_RATE = 0.01
SEARCH_RADIUS = 80
MAX_ITER = 5000
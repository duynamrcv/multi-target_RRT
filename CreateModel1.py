import math
import numpy as np

# Environment dimension
XDIM = 1000
YDIM = 800

X_RANGE = [0, XDIM]
Y_RANGE = [0, YDIM]

START = [540, 210]
GOALS = [[160, 320],
         [580, 710],
         [950, 350]]

# Obstacle
OBS_CIRCLE = [[730, 740, 80],
              [400, 300, 80],
              [170, 140, 90],
              [600, 100, 70],
              [940, 510, 80]]
OBS_RECTANGLE = [[740, 140, 170, 140],
                 [320, 570, 200, 220],
                 [580,420,120,150],
                 [120,410,170,120]]

# Robot constraints
ROBOT_RADIUS = 5
MAX_ANGLE = np.deg2rad(60.0)

# RRT configs
STEP_LENGTH = 100
GOAL_SAMPLE_RATE = 0.01
SEARCH_RADIUS = 80
MAX_ITER = 1000
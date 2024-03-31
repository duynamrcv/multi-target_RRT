import math
import numpy as np

# Environment dimension
XDIM = 1000
YDIM = 800

X_RANGE = [0, XDIM]
Y_RANGE = [0, YDIM]

START = [80, 80]
GOALS = [[880, 240],
         [ 80, 640],
         [720, 720]]

# Obstacle
OBS_CIRCLE = [[640,  80, 40],
              [800, 400, 40],
              [120, 440, 40],
              [240, 720, 40]]
OBS_RECTANGLE = [[160,   0,  40, 240],
                 [440,  80,  40, 200],
                 [600, 200, 220,  40],
                 [680, 240,  40, 160],
                 [ 80, 320, 240,  40],
                 [320, 320,  40, 180],
                 [360, 460, 200,  40],
                 [560, 460,  40, 200],
                 [760, 480, 240,  40],
                 [  0, 560, 240,  40],
                 [420, 560,  40, 240],
                 [600, 620, 240,  40]]

# Robot constraints
ROBOT_RADIUS = 10
MAX_ANGLE = np.deg2rad(60.0)

# RRT configs
STEP_LENGTH = 50
GOAL_SAMPLE_RATE = 0.01
SEARCH_RADIUS = 80
MAX_ITER = 5000
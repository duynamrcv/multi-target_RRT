import math
import numpy as np

# Environment dimension
XDIM = 1000
YDIM = 800

X_RANGE = [0, XDIM]
Y_RANGE = [0, YDIM]

START = [80, 80]
GOALS = [[ 80, 720],
         [800,  80],
         [960, 720]]

# Obstacle
OBS_CIRCLE = None
OBS_RECTANGLE = [[240,   0, 160,  80],
                 [640,   0,  80, 160],
                 [ 80, 160,  80,  80],
                 [400, 160, 160,  80],
                 [800, 160, 160,  80],
                 [240, 240,  80, 160],
                 [560, 240,  80, 160],
                 [ 80, 320,  80,  80],
                 [400, 400,  80,  80],
                 [640, 400, 320,  80],
                 [  0, 480, 160,  80],
                 [240, 480,  80, 160],
                 [640, 560,  80, 160],
                 [800, 560, 200,  80],
                 [160, 640,  80,  80],
                 [400, 640,  80, 160]]

# Robot constraints
ROBOT_RADIUS = 10
MAX_ANGLE = np.deg2rad(60.0)

# RRT configs
STEP_LENGTH = 50
GOAL_SAMPLE_RATE = 0.01
SEARCH_RADIUS = 80
MAX_ITER = 5000
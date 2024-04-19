import math
import numpy as np

# Environment dimension
XDIM = 1000
YDIM = 800

X_RANGE = [0, XDIM]
Y_RANGE = [0, YDIM]

START = [920, 80]
GOALS = [[70, 750],
         [50,  40],
         [790, 760]]

# Obstacle
OBS_CIRCLE = [[640,  80, 40],
              [800, 400, 40],
              [120, 440, 40],
              [240, 720, 40],
              [450, 470, 80],
              [690, 670, 80],
              [320, 120, 80],
              [780, 135, 60],
              [120, 245, 60],
              [132, 632, 60],
              [640, 290, 60],
              [440, 650, 60],
              [870, 550, 60],
              [640, 470, 40],
              [455, 250, 40],
              [260, 540, 60],
              [290, 320, 40],
              [900, 710, 40],
              [120, 85, 40],
              [880, 270, 80],
              [490, 110, 60]]

OBS_RECTANGLE = None

# Robot constraints
ROBOT_RADIUS = 10
MAX_ANGLE = np.deg2rad(60.0)

# RRT configs
STEP_LENGTH = 50
GOAL_SAMPLE_RATE = 0.01
SEARCH_RADIUS = 80
MAX_ITER = 5000
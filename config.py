import math

''' Robot constraint '''
rr = 5                          # robot radius
thangle = 60.0/180*math.pi      # angle constraint

''' RRT-RRTstar parameter '''
step_len = 100
goal_sample_rate = 0.01
search_radius = 80
iter_max = 1000
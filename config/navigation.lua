-- Map Dimensions --
dist_res = 0.1 -- 50 cm
dilation_factor = 3

-- Observation Model --
sigma_observation = 0.2
range_max = 5.0

-- Map Resize --
map_length_dist = 50
row_num = 2*(map_length_dist)/dist_res + 1

-- Pure Pursuit --
pursuit_radius = 0.75 --meters
goal_threshold = 0.5


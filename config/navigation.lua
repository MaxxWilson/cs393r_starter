-- Map Dimensions --
dist_res = 0.1 -- 50 cm
dilation_factor = 2

-- Observation Model --
sigma_observation = 0.2
range_max = 5.0

-- Map Resize --
map_length_dist = 50
row_num = 2*(map_length_dist)/dist_res + 1

-- NEW PARAMS --
length = 0.508 -- 20"
width = 0.2667 -- 10.5"
wheel_base = 0.32385 -- 12.75"
track_width = 0.235 -- 9.25
safety_margin = 0.05 -- 2"

dist_to_front_bumper = (length + wheel_base)/2 + safety_margin
dist_to_side_bumper = width/2 + safety_margin
dist_to_rear_bumper = (length-wheel_base)/2

min_curvature = -1.2
max_curvature = 1.2 -- +-0.75

 -- Dynamics --
max_acceleration = 6.0;
min_acceleration = -6.0;
max_velocity = 1.5;

-- Algorithmic Parameters --
safe_distance = 0.1 -- safe distance used in TOC control, stops with 5" left to obstacle
max_path_length = 8.0
curvature_increment = 0.02
num_curves = (max_curvature - min_curvature)/curvature_increment
clearance_factor = width/2 + safety_margin + 0.28 -- ~6"

sys_latency = 1e9 * 0.220; -- ns
sensing_latency =  1*sys_latency/4;
actuation_latency = 3*sys_latency/4;

----- Tunables -----
-- Obstace Avoidance --
clearance_gain = 7.0
dist_goal_gain = -0.7

-- Pure Pursuit --
pursuit_radius = 2.5 -- meters
goal_threshold = 0.5 -- meters
path_replan_deviation = 0.5
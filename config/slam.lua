-- Update Pose Thresholds --
dist_update_thresh = 0.3
angle_update_thresh = 0.2 -- 15°

-- CSM Search --
dist_res = 0.04 -- 0.02 -- 50 cm
theta_res = 0.02 -- 0.03 -- ~5°

theta_search_const = 13
dist_search_const = 5

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 5.0

-- Motion Model --

k1 = 1.0 -- 0.1    -- translation error from translation
k2 = 2.0 -- 0.1    -- translation error from rotation
k3 = 1.5 -- 0.05    -- rotation error from translation
k4 = 2.5 -- 0.2    -- rotation error from rotation

-- Observation Model --
gamma = 0.004
sigma_observation = 0.05 -- 0.2 
cost_map_min_prob = 1e-6 --

-- Map Resize --
resize_factor = 10
map_size = 10000
map_length_dist = dist_update_thresh + range_max + laser_offset + 4*sigma_observation + 5.0
row_num = 2*(map_length_dist)/dist_res + 1
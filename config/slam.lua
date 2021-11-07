-- Update Pose Thresholds --
dist_update_thresh = 0.3
angle_update_thresh = 0.5 -- 15°

-- CSM Search --
dist_res = 0.03 -- 0.02 -- 50 cm
theta_res = 0.01 -- 0.03 -- ~5°

theta_search_const = 20 
dist_search_const = 8

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 3.0

-- Motion Model --

k1 = 0.5 -- 0.1    -- translation error from translation
k2 = 1.5 -- 0.1    -- translation error from rotation
k3 = 0.5 -- 0.05    -- rotation error from translation
k4 = 2.0 -- 0.2    -- rotation error from rotation

-- Observation Model --
gamma = 0.001 -- 0.001
sigma_observation = 0.1 -- 0.2
resize_factor = 10 -- Divides scan

-- Map Resize --
map_size = 10000
map_length_dist = dist_update_thresh + range_max + laser_offset + 4*sigma_observation + 5.0
row_num = 2*(map_length_dist)/dist_res + 1
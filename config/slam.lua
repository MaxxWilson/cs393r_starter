-- Update Pose Thresholds --
dist_update_thresh = 0.3
angle_update_thresh = 0.1 -- 30°

-- CSM Search --
dist_res = 0.05 -- 50 cm
theta_res = 0.02 -- ~5°

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 8.0

-- Motion Model --

k1 = 0.1 --0.5 -- 0.1    -- translation error from translation
k2 = 0.1 --1.0 -- 0.1    -- translation error from rotation
k3 = 0.05 --0.1 -- 0.05    -- rotation error from translation
k4 = 0.2 --0.5 -- 0.2    -- rotation error from rotation

-- Observation Model --
gamma = 0.001
sigma_observation = 0.1 -- 0.2
resize_factor = 1 -- Divides scan

-- Map Resize --
map_size = 10000
map_length_dist = dist_update_thresh + range_max + laser_offset + 4*sigma_observation + 5.0
row_num = 2*(map_length_dist)/dist_res + 1
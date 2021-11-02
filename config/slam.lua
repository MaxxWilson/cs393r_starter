-- Update Pose Thresholds --
dist_update_thresh = 0.3
angle_update_thresh = 0.5 -- 30°

-- CSM Search --
dist_res = 0.03 -- 50 cm
theta_res = 0.01 -- ~5°

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 10.0

-- Motion Model --
k1 = 0.05 -- 0.1    -- translation error from translation
k2 = 0.06 -- 0.1    -- translation error from rotation
k3 = 0.05 -- 0.05    -- rotation error from translation
k4 = 0.06 -- 0.2    -- rotation error from rotation

-- Observation Model --
gamma = 0.1
sigma_observation = 0.2
resize_factor = 5 -- Divides scan

-- Map Resize --
map_size = 10000
map_length_dist = dist_update_thresh + range_max + laser_offset + 4*sigma_observation + 5.0
row_num = 2*(map_length_dist)/dist_res + 1
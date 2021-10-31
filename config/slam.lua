-- Update Pose Thresholds --
dist_update_thresh = 0.5
angle_update_thresh = 0.5236 -- 30°

-- CSM Search --
dist_res = 0.1 -- 10 cm
theta_res = 0.01 -- ~0.5°

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 10.0

-- Motion Model --
k1 = 0.2    -- translation error from translation
k2 = 0.3    -- translation error from rotation
k3 = 0.3    -- rotation error from translation
k4 = 0.5    -- rotation error from rotation

-- Observation Model --
gamma = 0.2
sigma_observation = 0.2

-- Map Resize --
map_size = 10000
map_length_dist = dist_update_thresh + range_max + laser_offset
row_num = 2*(dist_update_thresh + range_max + laser_offset)/dist_res + 1
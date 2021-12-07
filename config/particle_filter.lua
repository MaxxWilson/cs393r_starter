map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 9.5

-- Initial cloud distribution --
init_x_sigma = 0.3 -- 99% of the time, within a meter
init_y_sigma = 0.3 -- 99% of the time, within a meter
init_r_sigma = 0.3  -- 99% of the time, its within 35 deg

-- Motion Model Params --
k1 = 0.4   -- x error from translation         -- 95% of translations are within 15% margin of error
k2 = 0.0   -- x error from rotation            -- This is effectively zero given the small angle approx
k3 = 0.1   -- y error from translation         -- This is effectively zero given the small angle approx
k4 = 0.3   -- y error from rotation            -- at 45 deg, 99% of values within 10 cm
k5 = 0.5   -- rotation error from translation  -- at 1m, 99% of values within 7 deg
k6 = 1.5    -- rotation error from rotation     -- 95% of translations are within 15% margin of error

min_update_dist = 0.3                       -- Based on odometry messages at 1m/s at 40Hz
min_update_angle = 0.2

-- Limited by computation --
num_particles = 100 -- Increase until computation runs out
resize_factor = 20          -- # num_points / resize_factor = num_rays

sigma_observation = 0.1    -- Prof recommends 0.15-0.2 based on sensor specs
gamma = 0.005 -- 0.01                -- TODO Experimental tuning

-- Limits maximum weight error --
-- Increasing these makes it harsher on short/long errors for scan
dist_short = 0.23   -- 1 std from sensor 68.2%
dist_long = 0.28     -- 2 std from sensor 95%

resample_frequency = 8     -- TODO Experimental tuning

-- Cost Map --
dist_update_thresh = 0.3
angle_update_thresh = 0.2 -- 15°

-- CSM Search --

low_dist_res = 0.3 -- 0.05 -- 50 cm

dist_res = 0.03 -- 0.05 -- 50 cm
theta_res = 0.02 -- 0.02 -- ~5°
csm_eval_range_max =  5.0

theta_search_const = 10
dist_search_const = 20

map_length_dist = dist_update_thresh + range_max + laser_offset + 4*sigma_observation + 0.5
row_num = 2*(map_length_dist)/dist_res + 1

dilation_factor = 2
min_map_prob = -1e3
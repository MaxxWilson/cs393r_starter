map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

-- Initial cloud distribution, TODO tune from user error
init_x_sigma = 0.0
init_y_sigma = 0.0
init_r_sigma = 0.00

num_particles = 100 -- Prof recommends ~50

-- TODO Tune these from odometry messages and sampling time
k1 = 0.05 -- translation error from translation
k2 = 0.5 -- translation error from rotation
k3 = 0.01 -- rotation error from translation
k4 = 0.05 -- rotation error from rotation

laser_offset = 0.2 --TODO measure real car offset
resize_factor = 20 -- # num_points / resize_factor = num_rays

sigma_observation = 0.15    -- Prof recommends 0.15-0.2 based on sensor specs
gamma = 1/50               -- TODO Experimental tuning

dist_short = 0.1
dist_long = 0.1
range_min = 0.02
range_max = 10.0

min_dist_to_update = 0.01    -- TODO Tuning from odom messages and sampling time
resample_frequency = 10     -- TODO Experimental tuning
//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");
CONFIG_FLOAT(laser_offset, "laser_offset");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

// Yuhong
void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  
  scan.resize((int)(num_ranges / resize_factor));
  
  // Location of LIDAR in map frame
  Vector2f laser_loc = loc + Vector2f(CONFIG_laser_offset*cos(angle), CONFIG_laser_offset*sin(angle));
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) { // for each ray
    // Initialize the ray line
    line2f ray(0, 1, 2, 3);
    float ray_angle = angle + angle_min + resize_factor * i / num_ranges * (angle_max - angle_min);
    ray.p0[0] = laser_loc[0] + range_min * cos(ray_angle);
    ray.p0[1] = laser_loc[1] + range_min * sin(ray_angle);
    ray.p1[0] = laser_loc[0] + range_max * cos(ray_angle);
    ray.p1[1] = laser_loc[1] + range_max * sin(ray_angle);
    Vector2f final_intersection = laser_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    double min_dist = range_max;
    
    for (size_t i = 0; i < map_.lines.size(); ++i) { // for each line in map
      const line2f map_line = map_.lines[i];
      Vector2f intersection_point; // Return variable
      bool intersects = map_line.Intersection(ray, &intersection_point);
      if (intersects) {
        double cur_dist = (intersection_point - laser_loc).norm();
        if(cur_dist < min_dist) {
          final_intersection = intersection_point;
          min_dist = cur_dist;
        }
      }
    }

    scan[i] = final_intersection;
  }

  
}

// Yuhong
// Update the weight of the particle based on how well it fits the observation
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Get predicted point cloud
  Particle &particle = *p_ptr;
  vector<Vector2f> predicted_cloud;
  GetPredictedPointCloud(particle.loc, 
                         particle.angle, 
                         ranges.size(), 
                         range_min, 
                         range_max,
                         angle_min,
                         angle_max,
                         &predicted_cloud);
  // resize the ranges
  vector<float> trimmed_ranges(predicted_cloud.size());
  for(int i = 0; i < predicted_cloud.size(); i++) {
    trimmed_ranges[i] = ranges[i * resize_factor];
  }
  float particle_weight = 0;
  // Calculate the particle weight
  for(int i = 0; i < predicted_cloud.size(); i++) {
    float predicted_range = (predicted_cloud[i] - particle.loc).norm();
    float diff = abs(trimmed_ranges[i] - predicted_range);
    particle_weight += -diff; // smaller the diff, larger the particle weight
  }
  particle.weight = 0;
  particle.weight = particle_weight;
}

// Maxx
void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}
// Melissa
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  float x = rng_.Gaussian(0.0, 2.0);
  printf("Random number drawn from Gaussian distribution with 0 mean and "
         "standard deviation of 2 : %f\n", x);
}

// Maxx
void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter

///// Team Plan /////
// Predict (Melissa)
//    Given point and odometry msg, propogate point cloud forward
// Update (Yuhong)
//    predicted expected observation for a particle given map 
//    Compare observation with prediction
//    Assign weight to particle (Importance Weights?)
//    Note: Only update after car has travelled a certain distance?
// Resample (Maxx)
//    Given a set of weighted particles, resample probabilistically
//    Low-variance resampling
//    resample less often (every N updates)

// RECOMMENDED IMPLEMENTATION (Observation Likelihood Model)
// 1) start with simple observation likelihood model, pure gaussian (Log Likelihood?)
// 2) Test in simulator
// 3) Tune standard deviation, gamma, to prevent overconfident estimates
//
// 4) Next, implement robust piece-wise observation likelihood model
// 5) Tune params with logged data
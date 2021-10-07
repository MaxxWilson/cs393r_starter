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

CONFIG_INT(num_particles, "num_particles");
CONFIG_FLOAT(init_x_sigma, "init_x_sigma");
CONFIG_FLOAT(init_y_sigma, "init_y_sigma");
CONFIG_FLOAT(init_r_sigma, "init_r_sigma");
DEFINE_double(num_particles, 50, "Number of particles");
CONFIG_FLOAT(laser_offset, "laser_offset");

namespace particle_filter {
  bool new_first_odom;
  Vector2f first_odom_loc;
  float first_odom_angle;

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
  
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) { // for each ray
    // Initialize the ray line
    line2f ray(0, 1, 2, 3);
    float ray_angle = angle + angle_min + resize_factor * i / num_ranges * (angle_max - angle_min);
    ray.p0[0] = loc[0] + range_min * cos(ray_angle);
    ray.p0[1] = loc[1] + range_min * sin(ray_angle);
    ray.p1[0] = loc[0] + range_max * cos(ray_angle);
    ray.p1[1] = loc[1] + range_max * sin(ray_angle);
    Vector2f final_intersection = loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    double min_dist = range_max;
    
    for (size_t i = 0; i < map_.lines.size(); ++i) { // for each line in map
      const line2f map_line = map_.lines[i];
      Vector2f intersection_point; // Return variable
      bool intersects = map_line.Intersection(ray, &intersection_point);
      if (intersects) {
        double cur_dist = (intersection_point - loc).norm();
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
  Vector2f sensor_loc = BaseLinkToSensorFrame(particle.loc, particle.angle);
  GetPredictedPointCloud(sensor_loc, 
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
  float log_error = 0;
  // Calculate the particle weight
  for(int i = 0; i < predicted_cloud.size(); i++) {
    // Observation likelihood function, See Lecture 7, Slide 30
    // 1) Center gaussian on predicted range
    // 2) Evaluate gaussian at actual range to get likelihood
    // 3) Multiply all scan likelihoods together for total particle weight
    // *Note* Use log-likelihood to maintain precision
    float predicted_range = (predicted_cloud[i] - sensor_loc).norm();
    float diff = abs(trimmed_ranges[i] - predicted_range);
    log_error += std::pow(-Sq(diff) / vairance, gamma); // smaller the diff, larger the particle weight

  }
  particle.weight = 0;
  particle.weight = log_error;
}

// Maxx
void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  vector<Particle> new_particles(particles_.size());
  vector<double> weight_bins(particles_.size());
  
  // Calculate weight sum, get bins sized by particle weights as vector
  double weight_sum = 0;
  for(std::size_t i = 0; i < particles_.size(); i++){
    weight_sum += particles_[i].weight;
    weight_bins[i] = weight_sum;
  }

  // During resampling:
  for(std::size_t i = 0; i < particles_.size(); i++){
    double rand_weight = rng_.UniformRandom(0, weight_sum);
    auto new_particle_index = std::lower_bound(weight_bins.begin(), weight_bins.end(), rand_weight) - weight_bins.begin();
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size());
  }
  
  // After resampling:
  particles_ = new_particles;
}

// Maxx
void ParticleFilter::LowVarianceResample() {
  vector<Particle> new_particles(particles_.size());
  vector<double> weight_bins(particles_.size());
  
  // Calculate weight sum, get bins sized by particle weights as vector
  double weight_sum = 0;
  for(std::size_t i = 0; i < particles_.size(); i++){
    weight_sum += particles_[i].weight;
    weight_bins[i] = weight_sum;
  }

  double select_weight = rng_.UniformRandom(0, weight_sum);

  for(std::size_t i = 0; i < particles_.size(); i++){
    auto new_particle_index = std::lower_bound(weight_bins.begin(), weight_bins.end(), select_weight) - weight_bins.begin();
    select_weight = std::fmod(select_weight + weight_sum/((double) particles_.size()), weight_sum);
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = rng_.UniformRandom(); // 1/((double) particles_.size());
  }
  
  // After resampling:
  particles_ = new_particles;
}

void ParticleFilter::SetParticlesForTesting(vector<Particle> new_particles){
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  LowVarianceResample();
}

// Melissa
//TODO: empirically determine ks - drive the car and eval errors, 
//described in lect 9/15
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  // float d = sqrt(pow(odom_loc[0],2) + pow(odom_loc[1],2));
  // float r = d / (2*sin(odom_angle));
  // float arc_dist = r * odom_angle;
  if(new_first_odom){
    Vector2f first_odom_loc = odom_loc;
    float first_odom_angle = odom_angle;
    new_first_odom = false;
  }

  //Transform to map frame from odom frame 
  Eigen::Matrix2f R_odom;
  R_odom << cos(prev_odom_angle_ - first_odom_angle), sin(prev_odom_angle_ - first_odom_angle), -sin(prev_odom_angle_ - first_odom_angle), cos(prev_odom_angle_ - first_odom_angle);

  Eigen::Matrix2f R_map;
  R_map << cos(prev_odom_angle_ - 0), sin(prev_odom_angle_ - 0), -sin(prev_odom_angle_ - 0), cos(prev_odom_angle_ - 0);

  Eigen::Matrix2f T_odom_2;
  T_odom_2 << odom_loc[0] - first_odom_loc[0], odom_loc[1] - first_odom_loc[1];

  Eigen::Matrix2f T_odom_1;
  T_odom_1 << prev_odom_loc_[0] - first_odom_loc[0], prev_odom_loc_[1] - first_odom_loc[1];

  Eigen::Matrix2f T_map_1;
  T_map_1 << prev_odom_loc_[0] - 0, prev_odom_loc_[1] - 0;

  Eigen::Matrix2f del_T_base;
  del_T_base << R_odom * (T_odom_2 -T_odom_1);

  Eigen::Matrix2f T_map_2;
  T_map_2 << T_map_1 + R_map * del_T_base;
 
  int k1 = 0.5; int k2 = 0.5; int k3 = 0.5; int k4 = 0.5;
  for(Particle &particle: particles_){
    float x_d = particle.loc[0] + odom_loc[0] - prev_odom_loc_[0] ;
    float y_d = particle.loc[1] + odom_loc[1] - prev_odom_loc_[1];
    float tht_d = particle.angle + odom_angle - prev_odom_angle_;
    // std::cout <<"odom_diff_x: " << odom_loc[0] - prev_odom_loc_[0] << "\n";
    // std::cout <<"odom_diff_y: " << odom_loc[1] - prev_odom_loc_[1] << "\n";
    std::cout <<"odom " << odom_loc << "\n";
    // std::cout <<"x_d: " << x_d<< "\n";
    // std::cout <<"y_d: " << y_d<< "\n";
    //  std::cout <<"particle loc: " << particle.loc<< "\n";
    float sigma_xy = k1 * sqrt(pow(x_d, 2) + pow(y_d, 2)) + k2 * abs(odom_angle - prev_odom_angle_ + particle.angle); // x and y
    float sigma_tht = k3 * sqrt(pow(x_d, 2) + pow(y_d, 2)) + k4 * abs(odom_angle - prev_odom_angle_ + particle.angle); // tht
    // std::cout <<"sigma_xy: " << sigma_xy<< "\n";
    float e_xy = rng_.Gaussian(0.0, sigma_xy);
    float e_tht = rng_.Gaussian(0.0, sigma_tht);
    std::cout <<"e_xy: " << e_xy<< "\n";
    //implement for all of previous locations in distribution
    particle.loc[0] = x_d + e_xy;
    particle.loc[1] = y_d + e_xy;
    particle.angle = tht_d + e_tht;
  }

  prev_odom_loc_[0] = odom_loc[0];
  prev_odom_loc_[1] = odom_loc[1];
  prev_odom_angle_ = odom_angle;

   // TransformToMap(odom_loc, odom_angle);
  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  
}

void TransformToMap(const Vector2f& odom_loc,
                     const float odom_angle){
      


}

// Maxx
void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

  particles_.resize(CONFIG_num_particles);

  for(Particle &particle: particles_){
    particle.loc = Eigen::Vector2f(
      loc[0] + rng_.Gaussian(0, CONFIG_init_x_sigma),
      loc[1] + rng_.Gaussian(0, CONFIG_init_y_sigma)
      );
    particle.angle = rng_.Gaussian(angle, CONFIG_init_r_sigma);
    particle.weight = 1/((double)particles_.size());
  }
  new_first_odom = true;
  map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  double weight_sum = 0;
  for(Particle particle: particles_){
    loc += particle.loc * particle.weight;
    angle += particle.angle * particle.weight;
    weight_sum += particle.weight;
  }

  loc /= weight_sum;
  angle /= weight_sum;
}

Eigen::Vector2f ParticleFilter::BaseLinkToSensorFrame(const Eigen::Vector2f &loc, const float &angle){
  return loc + Vector2f(CONFIG_laser_offset*cos(angle), CONFIG_laser_offset*sin(angle));
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
//    TODO: Importance Sampling

// RECOMMENDED IMPLEMENTATION (Observation Likelihood Model)
// 1) start with simple observation likelihood model, pure gaussian (Log Likelihood?)
// 2) Test in simulator
// 3) Tune standard deviation, gamma, to prevent overconfident estimates
//
// 4) Next, implement robust piece-wise observation likelihood model
// 5) Tune params with logged data
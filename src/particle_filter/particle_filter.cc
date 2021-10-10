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

CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");

CONFIG_FLOAT(laser_offset, "laser_offset");

CONFIG_FLOAT(min_dist_to_update, "min_dist_to_update");
CONFIG_DOUBLE(sigma_observation, "sigma_observation");
CONFIG_DOUBLE(gamma, "gamma");
CONFIG_DOUBLE(dist_short, "dist_short");
CONFIG_DOUBLE(dist_long, "dist_long");
CONFIG_DOUBLE(range_min, "range_min");
CONFIG_DOUBLE(range_max, "range_max");

CONFIG_DOUBLE(resize_factor, "resize_factor");
CONFIG_INT(resample_frequency, "resample_frequency");

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

  // Note: The returned values must be set using the `scan` variable:=

  scan.resize((int)(num_ranges / CONFIG_resize_factor));
  
  Vector2f sensor_loc = BaseLinkToSensorFrame(loc, angle);

  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) { // for each ray
    // Initialize the ray line
    line2f ray(0, 1, 2, 3);
    float ray_angle = angle + angle_min + CONFIG_resize_factor * i / num_ranges * (angle_max - angle_min);
    ray.p0[0] = sensor_loc[0] + range_min * cos(ray_angle);
    ray.p0[1] = sensor_loc[1] + range_min * sin(ray_angle);
    ray.p1[0] = sensor_loc[0] + range_max * cos(ray_angle);
    ray.p1[1] = sensor_loc[1] + range_max * sin(ray_angle);
    Vector2f final_intersection = sensor_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    double min_dist = range_max;
    
    for (size_t i = 0; i < map_.lines.size(); ++i) { // for each line in map
      const line2f map_line = map_.lines[i];
      Vector2f intersection_point; // Return variable
      bool intersects = map_line.Intersection(ray, &intersection_point);
      if (intersects) {
        double cur_dist = (intersection_point - sensor_loc).norm();
        if(cur_dist < min_dist) {
          final_intersection = intersection_point;
          min_dist = cur_dist;
        }
      }
    }

    scan[i] = final_intersection;
  }

  
}

double GetRobustObservationLikelihood(double measured, double expected, double dist_short, double dist_long){
  
  if(measured < CONFIG_range_min || measured > CONFIG_range_max){

  }
  else if(measured < (expected - dist_short)){
    return dist_short;
  }
  else if(measured > (expected + dist_long)){
    return dist_long;
  }
  else{
    return measured - expected;
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
  vector<Vector2f> predicted_cloud; // map frame
  GetPredictedPointCloud(particle.loc, 
                         particle.angle, 
                         ranges.size(), 
                         range_min, 
                         range_max,
                         angle_min,
                         angle_max,
                         &predicted_cloud);
  Vector2f sensor_loc = BaseLinkToSensorFrame(particle.loc, particle.angle);
  // resize the ranges
  vector<float> trimmed_ranges(predicted_cloud.size());
  for(std::size_t i = 0; i < predicted_cloud.size(); i++) {
    trimmed_ranges[i] = ranges[i * CONFIG_resize_factor];
  }
  double log_error = 0;
  // Calculate the particle weight
  for(std::size_t i = 0; i < predicted_cloud.size(); i++) {
    double predicted_range = (predicted_cloud[i] - sensor_loc).norm();
    //double diff = GetRobustObservationLikelihood(trimmed_ranges[i], predicted_range, CONFIG_dist_short, CONFIG_dist_long);
    double diff = trimmed_ranges[i] - predicted_range;
    log_error += -CONFIG_gamma * Sq(diff) / Sq(CONFIG_sigma_observation);

  }
  particle.weight = log_error;
}

// Maxx
void ParticleFilter::Resample() {
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

  double select_weight = rng_.UniformRandom(0, weight_sum_);

  for(std::size_t i = 0; i < particles_.size(); i++){
    int new_particle_index = std::lower_bound(weight_bins_.begin(), weight_bins_.end(), select_weight) - weight_bins_.begin();
    select_weight = std::fmod(select_weight + weight_sum_/((double) particles_.size()), weight_sum_);
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size()); // rng_.UniformRandom(); good for testing
  }
  weight_sum_ = 1.0;
  
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
  if((last_update_loc_ - prev_odom_loc_).norm() > CONFIG_min_dist_to_update){

    max_weight_log_ = -1e10; // Should be smaller than any
    weight_sum_ = 0;
    weight_bins_.resize(particles_.size());
    std::fill(weight_bins_.begin(), weight_bins_.end(), 0);

    // Update each particle with log error weight and find largest weight (smallest negative number)
    for(Particle &p: particles_){
      Update(ranges, range_min, range_max, angle_min, angle_max, &p);
      max_weight_log_ = std::max(max_weight_log_, p.weight);
    }

    // cout << endl << endl;
    // Normalize log-likelihood weights by max log weight and transform back to linear scale
    // Sum all linear weights and generate bins
    for(std::size_t i = 0; i < particles_.size(); i++){
      particles_[i].weight = exp(particles_[i].weight - max_weight_log_);
      weight_sum_ += particles_[i].weight;
      weight_bins_[i] = weight_sum_;
    }

    if(!(resample_loop_counter_ % CONFIG_resample_frequency)){
      LowVarianceResample();
    }
    last_update_loc_ = prev_odom_loc_;
    resample_loop_counter_++;       
  }                      
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

  //Transform to map frame from odom frame
  auto R_odom = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
  Eigen::Vector2f del_T_base = R_odom * (odom_loc - prev_odom_loc_);

  for(Particle &particle: particles_){
    
    auto R_map = Eigen::Rotation2D<float>(particle.angle).toRotationMatrix();

    float sigma_x = CONFIG_k1 * del_T_base.norm() + CONFIG_k2 * abs(math_util::AngleDiff(odom_angle, prev_odom_angle_)); // x and y
    float sigma_y = CONFIG_k1 * del_T_base.norm() + CONFIG_k2 * abs(math_util::AngleDiff(odom_angle, prev_odom_angle_));
    float sigma_tht = CONFIG_k3 * del_T_base.norm() + CONFIG_k4 * abs(math_util::AngleDiff(odom_angle, prev_odom_angle_)); // tht
    
    float e_x = rng_.Gaussian(0.0, sigma_x);
    float e_y = rng_.Gaussian(0.0, sigma_y);
    float e_tht = rng_.Gaussian(0.0, sigma_tht);
    
    //implement for all of previous locations in distribution
    particle.loc += R_map * del_T_base + Eigen::Vector2f(e_x, e_y);
    particle.angle += odom_angle - prev_odom_angle_ + e_tht;
  }

  prev_odom_loc_[0] = odom_loc[0];
  prev_odom_loc_[1] = odom_loc[1];
  prev_odom_angle_ = odom_angle;
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
    particle.angle = angle; //rng_.Gaussian(angle, CONFIG_init_r_sigma);
    particle.weight = 1/((double)particles_.size());
  }
  max_weight_log_ = 0;
  last_update_loc_ = prev_odom_loc_;
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

  Eigen::Vector2f angle_point = Eigen::Vector2f(0, 0);
  for(Particle particle: particles_){
    loc += particle.loc * particle.weight;
    angle_point += Eigen::Vector2f(cos(particle.angle), sin(particle.angle)) * particle.weight;
  }

  loc /= weight_sum_;
  angle_point /= weight_sum_;
  angle = atan2(angle_point[1], angle_point[0]);
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
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

#include "config_reader/config_reader.h"
#include "particle_filter.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

namespace particle_filter {

// Initial Parameters
CONFIG_INT(num_particles, "num_particles");
CONFIG_FLOAT(init_x_sigma, "init_x_sigma");
CONFIG_FLOAT(init_y_sigma, "init_y_sigma");
CONFIG_FLOAT(init_r_sigma, "init_r_sigma");

// Motion Model Parameters
CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");
CONFIG_FLOAT(k5, "k5");
CONFIG_FLOAT(k6, "k6");

// LIDAR Geometry and Specs
CONFIG_FLOAT(laser_offset, "laser_offset");
CONFIG_DOUBLE(range_min, "range_min");
CONFIG_DOUBLE(range_max, "range_max");

// Minimum motion for particle filter update
CONFIG_FLOAT(min_update_dist, "min_update_dist");
CONFIG_FLOAT(min_update_angle, "min_update_angle");

// Observation Likelihood Model Parameters
CONFIG_DOUBLE(sigma_observation, "sigma_observation");
CONFIG_DOUBLE(gamma, "gamma");
CONFIG_DOUBLE(dist_short, "dist_short");
CONFIG_DOUBLE(dist_long, "dist_long");
CONFIG_DOUBLE(resize_factor, "resize_factor");

// Resample Frequency
CONFIG_INT(resample_frequency, "resample_frequency");

// CSM Lookup Table Parameters
CONFIG_FLOAT(low_dist_res, "low_dist_res");
CONFIG_FLOAT(low_theta_res, "low_theta_res");

CONFIG_FLOAT(dist_res, "dist_res");
CONFIG_FLOAT(theta_res, "theta_res");

CONFIG_FLOAT(theta_search_range, "theta_search_range");
CONFIG_FLOAT(dist_search_range, "dist_search_range");

CONFIG_DOUBLE(csm_sigma_observation, "csm_sigma_observation");
CONFIG_DOUBLE(csm_gamma, "csm_gamma");

CONFIG_DOUBLE(map_length_dist, "map_length_dist");
CONFIG_DOUBLE(min_map_prob, "min_map_prob");
CONFIG_DOUBLE(csm_eval_range_max, "csm_eval_range_max");
CONFIG_INT(csm_resize, "csm_resize");

CONFIG_STRING(localization_mode, "localization_mode");

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}


void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

cv::Mat ParticleFilter::GetTFCubeImage(){
  return likelihood_cube_.GetImage();
}

cv::Mat ParticleFilter::GetLowResTFCubeImage(){
  return low_res_likelihood_cube_.GetImage();
}

cv::Mat ParticleFilter::GetEKFProposalImage(){
  return ekf_proposal_img.GetImage();
}

cv::Mat ParticleFilter::GetLidarProposalImage(){
  return lidar_proposal_img.GetImage();
}

cv::Mat ParticleFilter::GetOdomProposalImage(){
  return odom_proposal_img.GetImage();
}

Eigen::Vector2f ParticleFilter::BaseLinkToSensorFrame(const Eigen::Vector2f &loc, const float &angle){
  return loc + Vector2f(CONFIG_laser_offset*cos(angle), CONFIG_laser_offset*sin(angle));
}

csm_map::CSMMap ParticleFilter::GetCSMMap(){
  return csm_map_;
}

csm_map::CSMMap ParticleFilter::GetLRCSMMap(){
  return low_csm_map_;
}

bool ParticleFilter::horizontal_line_compare(const geometry::line2f l1, const geometry::line2f l2){
  return l1.p0.y() < l2.p0.y();
}

bool ParticleFilter::vertical_line_compare(const geometry::line2f l1, const geometry::line2f l2){
  return l1.p0.x() < l2.p0.x();
}

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

  // Skip a set ratio of laser scans in evaluating LIDAR readings
  scan.resize((int)(num_ranges / CONFIG_resize_factor));
  
  // Get sensor location in world frame
  Vector2f sensor_loc = BaseLinkToSensorFrame(loc, angle);
  
  // Locate robot within horizontal and vertical grid formed from map lines
  int v_start_index = std::lower_bound(horizontal_lines_.begin(), horizontal_lines_.end(), line2f(sensor_loc, sensor_loc), horizontal_line_compare) - horizontal_lines_.begin();
  int h_start_index = std::lower_bound(vertical_lines_.begin(), vertical_lines_.end(), line2f(sensor_loc, sensor_loc), vertical_line_compare) - vertical_lines_.begin();

  // Return if no map is loaded
  if(!vertical_lines_.size() || !horizontal_lines_.size()){
    return;
  }

  // for each ray
  for (size_t i = 0; i < scan.size(); ++i){
    // Initialize the ray line
    float ray_angle = angle + angle_min + CONFIG_resize_factor * i / num_ranges * (angle_max - angle_min);
    float C0 = cos(ray_angle);
    float S0 = sin(ray_angle);

    Vector2f final_intersection = sensor_loc + CONFIG_range_max * Vector2f(C0, S0);
    line2f ray(sensor_loc, final_intersection);

    // Determine horizontal and vertical search directions
    int h_dir = math_util::Sign(ray.Dir().x());
    int v_dir = math_util::Sign(ray.Dir().y());

    std::size_t v_search_index = v_start_index;
    std::size_t h_search_index = h_start_index;

    if(h_dir < 0){
      h_search_index += 1;
    }

    if(v_dir > 0){
      v_search_index += 1;
    }

    // Search through ordered map lines, from robot location in direction of ray, until a collision is found or max range met
    Vector2f final_intersection_xy = final_intersection;
    bool intersection_found = false;
    double curr_dist = 0;
    while(!intersection_found && curr_dist < ray.Length()){
      float xi = (h_search_index < vertical_lines_.size()) ? abs(vertical_lines_[h_search_index].p0.x() - ray.p0.x()) : 100;
      float yi = (v_search_index < horizontal_lines_.size()) ? abs(horizontal_lines_[v_search_index].p0.y() - ray.p0.y()) : 100;

      float rx = abs(xi / C0);
      float ry = abs(yi / S0);

        if(rx < ry && rx < ray.Length()){
          curr_dist = rx;
          intersection_found = ray.Intersection(vertical_lines_[h_search_index], &final_intersection_xy);
          h_search_index += h_dir;
        }
        else if(ry <= rx && ry < ray.Length()){
          curr_dist = ry;
          intersection_found = ray.Intersection(horizontal_lines_[v_search_index], &final_intersection_xy);
          v_search_index += v_dir;
        }
        else{
          final_intersection_xy = final_intersection;
          break;
        }
    }

    // Search any angled lines (unsortable) in the map directly for this ray
    float curr_dist_angled = range_max;
    Vector2f final_intersection_angled = final_intersection;
    for (size_t i = 0; i < angled_lines_.size(); ++i) {
      if(ray.Intersection(angled_lines_[i], &final_intersection_angled)){
        float new_dist = (final_intersection_angled - ray.p0).norm();
        if(new_dist < curr_dist_angled){
          curr_dist_angled = new_dist;
        }
      }     
    }

    // Select the closest collision point
    if((final_intersection_angled - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_angled;
    }
    if((final_intersection_xy - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_xy;
    }

    scan[i] = final_intersection;
  }
}

double GetRobustObservationLikelihood(double measured, double expected, double dist_short, double dist_long){
  
  if(measured < CONFIG_range_min || measured > CONFIG_range_max){
    return 0;
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

void ParticleFilter::Predict(const Eigen::Vector2f& odom_loc, const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // propagate particles forward based on odometry.

  // // rotation matrix from last odom to last baselink
  // auto rot_odom1_to_bl1 = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
  
  // // Change in translation and angle from odometry
  // Eigen::Vector2f delta_translation = rot_odom1_to_bl1 * (odom_loc - prev_odom_loc_);
  // float delta_angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);

  for(Particle &particle: particles_){

    Eigen::Vector2f e_xy = Eigen::Vector2f((float) rng_.Gaussian(0.0, estimated_odom_.covariance(0,0)),(float) rng_.Gaussian(0.0, estimated_odom_.covariance(1,1)));

    // Transform noise to Base Link 1 using estimated angle to get noisy translation
    auto rot_b2_to_b1 = Eigen::Rotation2D<float>(estimated_odom_.pose.angle).toRotationMatrix();
    Eigen::Vector2f noisy_translation = estimated_odom_.pose.translation + rot_b2_to_b1 * e_xy; // in previous base_link
    
    // Transform noise to map using current particle angle
    auto rot_bl1_to_map = Eigen::Rotation2D<float>(particle.angle).toRotationMatrix();
    particle.loc += rot_bl1_to_map * noisy_translation;   
    //particle.angle += noisy_angle;        
  }

  // Update previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  // double d_translation = (last_update_loc_ - prev_odom_loc_).norm();
  // double d_angle = math_util::AngleDiff(last_update_angle_, prev_odom_angle_);

  // std::cout << "Motion Model: " << d_translation << ", " << d_angle << std::endl;
}

/**
 * @brief Predicts EKF distribution when there is new Odometry data 
 * 
 * @param odom_loc 
 * @param odom_angle 
 */
void ParticleFilter::PredictEKF(const Eigen::Vector2f& odom_loc, const float odom_angle){

  if(!odom_initialized_){
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  // rotation matrix from last odom to last baselink
  auto rot_odom1_to_bl1 = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
  
  // Change in translation and angle from odometry
  Eigen::Vector2f delta_translation = rot_odom1_to_bl1 * (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);

  // Get translation noise in Base Link 2
  float sigma_x = CONFIG_k1 * delta_translation.norm() + CONFIG_k2 * abs(delta_angle);
  float sigma_y = CONFIG_k3 * delta_translation.norm() + CONFIG_k4 * abs(delta_angle);
  float sigma_tht = CONFIG_k5 * delta_translation.norm() + CONFIG_k6 * abs(delta_angle);

  // Transform translation noise to Base Link 1 using estimated angle to get noisy translation
  auto rot_b2_to_b1 = Eigen::Rotation2D<float>(delta_angle).toRotationMatrix();
  Eigen::Matrix2f bl2_covariance;

  bl2_covariance << math_util::Pow(sigma_x, 2),        0,
                    0,                                 math_util::Pow(sigma_y, 2);

  Eigen::Matrix2f bl1_cov = rot_b2_to_b1*bl2_covariance*rot_b2_to_b1.transpose();

  //sigma_x, sigma_y, sigma_tht are calculated similar to PF Predict function
  Q <<    bl1_cov(0, 0),                          bl1_cov(0, 1),                            0,
          bl1_cov(1, 0)                        , bl1_cov(1, 1),                            0,
          0                         ,                          0, math_util::Pow(sigma_tht, 2);

  // //sigma_x, sigma_y, sigma_tht are calculated similar to PF Predict function
  // Q <<    math_util::Pow(bl1_sigma.x(), 2),                          0,                            0,
  //         0                         , math_util::Pow(bl1_sigma.y(), 2),                            0,
  //         0                         ,                          0, math_util::Pow(sigma_tht, 2);

  wheel_odom_.pose.ApplyPose(Pose2D<float>(delta_angle, delta_translation));
  wheel_odom_.covariance += Q;

  if(std::strcmp(CONFIG_localization_mode.c_str(), "odom") == 0){
    // Update odometry estimate
    estimated_odom_.pose = wheel_odom_.pose;
    estimated_odom_.covariance = wheel_odom_.covariance;
    std::cout << "X/Y/Theta STD: " << sqrt(estimated_odom_.covariance(0, 0)) << ", " << sqrt(estimated_odom_.covariance(1, 1)) << ", " << sqrt(estimated_odom_.covariance(2, 2)) << std::endl;
  }

  // std::cout << "Odometry: " << delta_translation.x() << ", " << delta_translation.y() << ", " << delta_angle << std::endl;
  // std::cout << "wheel_odom_: " << wheel_odom_.pose.translation.x() << ", " << wheel_odom_.pose.translation.y() << ", " << wheel_odom_.pose.angle << std::endl;

  // Update last odom
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

/**
 * @brief Updates EKF distribution when there is new LIDAR/Odometry data 
 * Distribution generated is of transforms from previous location to next predicted location
 * Particles in PF then sample transforms from combined EKF and PF motion model distributions
 * Odometry and Lidar distrbutions are fused during has_new_lidar block
 */
void ParticleFilter::UpdateEKF(){
  odom_proposal_img = transform_cube_slice::TransformCubeSlice(0.5, 0.01, 00);
  lidar_proposal_img = transform_cube_slice::TransformCubeSlice(0.5, 0.01, 00);
  ekf_proposal_img = transform_cube_slice::TransformCubeSlice(0.5, 0.01, 00);

  //Kalman Gain K - wheel_odom_.covariance = Q, lidar_odom_.covariance = R
  //K = Q * (Q+R)^-1
  Eigen::Matrix3f K = wheel_odom_.covariance * (wheel_odom_.covariance + lidar_odom_.covariance).inverse();

  //New mean of combined Predicted state and Sensor state distributions
  //mean_state is mean of predicted distribution uq is mean of sensor distribution
  //ut = uq + K * (ur -uq)
  Eigen::Vector3f ut = wheel_odom_.GetStateVector() + K * (lidar_odom_.GetStateVector() - wheel_odom_.GetStateVector());
  estimated_odom_.pose.ApplyPose(Pose2D<float>(ut[2], Eigen::Vector2f(ut[0], ut[1])));

  //New COV of combined Predicted state and Sensor state distributions
  //COVt = Q - K * Q
  estimated_odom_.covariance = wheel_odom_.covariance - K * wheel_odom_.covariance;

  // std::cout << "Wheel Odometry: " << wheel_odom_.pose.translation.x() << ", " << wheel_odom_.pose.translation.y() << ", " << wheel_odom_.pose.angle << std::endl;
  // std::cout << "X/Y/Theta STD: " << sqrt(wheel_odom_.covariance(0, 0)) << ", " << sqrt(wheel_odom_.covariance(1, 1)) << ", " << sqrt(wheel_odom_.covariance(2, 2)) << std::endl;
  
  // std::cout << "Lidar Odometry: " << lidar_odom_.pose.translation.x() << ", " << lidar_odom_.pose.translation.y() << ", " << lidar_odom_.pose.angle << std::endl;
  // std::cout << "X/Y/Theta STD: " << sqrt(lidar_odom_.covariance(0, 0)) << ", " << sqrt(lidar_odom_.covariance(1, 1)) << ", " << sqrt(lidar_odom_.covariance(2, 2)) << std::endl;

  // std::cout << "Estimated Odometry: " << estimated_odom_.pose.translation.x() << ", " << estimated_odom_.pose.translation.y() << ", " << estimated_odom_.pose.angle << std::endl;
  // std::cout << "X/Y/Theta STD: " << sqrt(estimated_odom_.covariance(0, 0)) << ", " << sqrt(estimated_odom_.covariance(1, 1)) << ", " << sqrt(estimated_odom_.covariance(2, 2)) << std::endl << std::endl;

  cv::Mat odom_img = odom_proposal_img.GetImage();
  cv::Mat lidar_img = lidar_proposal_img.GetImage();
  cv::Mat ekf_img = ekf_proposal_img.GetImage();

  for(int x = 0; x < ekf_proposal_img.GetColNum(); x++){
    for(int y = 0; y < ekf_proposal_img.GetColNum(); y++){
      int image_y = ekf_proposal_img.GetColNum() - y - 1;
      double x_distance = x*0.01-0.25;
      double y_distance = y*0.01-0.25;

      double wheel_odom_prob = exp(-0.5*(wheel_odom_.pose.translation.x() - x_distance)*(wheel_odom_.pose.translation.x() - x_distance)/wheel_odom_.covariance(0, 0))
                              *exp(-0.5*(wheel_odom_.pose.translation.y() - y_distance)*(wheel_odom_.pose.translation.y() - y_distance)/wheel_odom_.covariance(1, 1));

      double lidar_odom_prob = exp(-0.5*(lidar_odom_.pose.translation.x() - x_distance)*(lidar_odom_.pose.translation.x() - x_distance)/lidar_odom_.covariance(0, 0))
                              *exp(-0.5*(lidar_odom_.pose.translation.y() - y_distance)*(lidar_odom_.pose.translation.y() - y_distance)/lidar_odom_.covariance(1, 1));

      double ekf_odom_prob = exp(-0.5*(estimated_odom_.pose.translation.x() - x_distance)*(estimated_odom_.pose.translation.x() - x_distance)/estimated_odom_.covariance(0, 0))
                              *exp(-0.5*(estimated_odom_.pose.translation.y() - y_distance)*(estimated_odom_.pose.translation.y() - y_distance)/estimated_odom_.covariance(1, 1));

      odom_img.at<cv::Vec3b>(image_y, x)[0] = 255 - (int) (255*wheel_odom_prob);
      odom_img.at<cv::Vec3b>(image_y, x)[1] = 255 - (int) (255*wheel_odom_prob);
      odom_img.at<cv::Vec3b>(image_y, x)[2] = 255;

      lidar_img.at<cv::Vec3b>(image_y, x)[0] = 255;
      lidar_img.at<cv::Vec3b>(image_y, x)[1] = 255 - (int) (255*lidar_odom_prob);
      lidar_img.at<cv::Vec3b>(image_y, x)[2] = 255 - (int) (255*lidar_odom_prob);

      ekf_img.at<cv::Vec3b>(image_y, x)[0] = 255 - (int) (255*ekf_odom_prob);
      ekf_img.at<cv::Vec3b>(image_y, x)[1] = 255;
      ekf_img.at<cv::Vec3b>(image_y, x)[2] = 255 - (int) (255*ekf_odom_prob);
    }
  }
}

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

  // Get sensor location in global frame
  Vector2f sensor_loc = BaseLinkToSensorFrame(particle.loc, particle.angle);
  
  // resize the laser scan ranges
  vector<float> trimmed_ranges(predicted_cloud.size());
  
  // Calculate the particle weight in log likelihood
  particle.weight = 0;
  for(std::size_t i = 0; i < predicted_cloud.size(); i++) {
    trimmed_ranges[i] = ranges[i * CONFIG_resize_factor];
    double predicted_range = (predicted_cloud[i] - sensor_loc).norm();
    double diff = GetRobustObservationLikelihood(trimmed_ranges[i], predicted_range, CONFIG_dist_short, CONFIG_dist_long);
    particle.weight += -CONFIG_gamma * Sq(diff) / Sq(CONFIG_sigma_observation);
  } 
}

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

void ParticleFilter::LowVarianceResample() {
  vector<Particle> new_particles(particles_.size());

  double select_weight = rng_.UniformRandom(0, weight_sum_);

  for(std::size_t i = 0; i < particles_.size(); i++){
    int new_particle_index = std::lower_bound(weight_bins_.begin(), weight_bins_.end(), select_weight) - weight_bins_.begin();
    select_weight = std::fmod(select_weight + weight_sum_/((double) particles_.size()), weight_sum_);
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size());
  }
  weight_sum_ = 1.0;
  
  // After resampling:
  particles_ = new_particles;
}

void ParticleFilter::SetParticlesForTesting(vector<Particle> new_particles){
  particles_ = new_particles;
}

void ParticleFilter::ConvertScanToPointCloud(const float angle_min, const float angle_increment, const std::vector<float>& ranges, std::vector<Eigen::Vector2f> &cloud){
  for(std::size_t i = 0; i < ranges.size(); i++){
    // Polar to Cartesian conversion, transforms to base link frame of new pose
    float angle = angle_min + angle_increment*i;
    float x = ranges[i]*cos(angle) + CONFIG_laser_offset;
    float y = ranges[i]*sin(angle);
    scan_cloud_[i] = Vector2f(x, y);
  }
}
void ParticleFilter::InitializeCovariance() {
  mean_odom_[0] = 0;
  mean_odom_[1] = 0;
  mean_angle_point_ = Eigen::Vector2f(0, 0);
  K_ << 0, 0, 0,
       0, 0, 0,
       0, 0, 0;
  s_ = 0;
}

void ParticleFilter::ComputeCovariance(Eigen::Vector2f odom, float angle, float prob) {
  mean_odom_ += odom * prob;
  mean_angle_point_ += Eigen::Vector2f(cos(angle), sin(angle)) * prob;
  Eigen::Vector3f xi;
  xi << odom.x(), odom.y(), angle;
  K_ += xi * xi.transpose() * prob;
  s_ += prob;
}

Eigen::Matrix3f ParticleFilter::ComputeCovariance() {
  Eigen::Matrix3f res;
  Eigen::Vector3f mean_state;
  mean_state << mean_odom_.x(), mean_odom_.y(), atan2(mean_angle_point_[1], mean_angle_point_[0]);
  res = 1/s_ * K_ - 1/pow(s_, 2) * mean_state * mean_state.transpose();
  return res;
}

void ParticleFilter::EstimateLidarOdometry(){
    float theta_low = -CONFIG_theta_search_range;
    float theta_high = CONFIG_theta_search_range;
    float translation_low = -CONFIG_dist_search_range;
    float translation_high = CONFIG_dist_search_range;

    low_res_likelihood_cube_ = transform_cube_slice::TransformCubeSlice((2*CONFIG_dist_res + 2*CONFIG_dist_search_range)/2, CONFIG_low_dist_res, -1e5);
    likelihood_cube_ = transform_cube_slice::TransformCubeSlice((2*CONFIG_dist_res + 2*CONFIG_dist_search_range)/2, CONFIG_dist_res, -1e5);

    auto low_res_queue = std::vector<SearchRegion>();
    
    double low_P = -1e10;   // Log Probability of pose
    Pose2D<float> low_T;
    Pose2D<float> T;

    Eigen::Matrix3f covariance;

    InitializeCovariance();

    // For each change in theta
    for(int angle_index = 0; angle_index <= (theta_high - theta_low) / CONFIG_low_theta_res; angle_index++){
      float dtheta = theta_low + angle_index * CONFIG_low_theta_res;
      float angle_diff = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.angle + dtheta, CONFIG_low_theta_res);
      
      // Low Resolution
      for(int x_index = 0; x_index <= (translation_high - translation_low) / CONFIG_low_dist_res; x_index++){
        float dx = translation_low + x_index * CONFIG_low_dist_res;
        Vector2f trans_diff;
        trans_diff[0] = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.translation.x() + dx, CONFIG_low_dist_res);
        // For each possible Y
        for(int y_index = 0; y_index <= (translation_high - translation_low) / CONFIG_low_dist_res; y_index++){
          float dy = translation_low + y_index * CONFIG_low_dist_res;
          trans_diff[1] = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.translation.y() + dy, CONFIG_low_dist_res);

          double pose_log_prob = 0.0;

          // For each point in scan
          for(std::size_t i = 0; i < scan_cloud_.size(); i+=CONFIG_csm_resize){

            // Transform new point into frame of previous scan / reference image
            Vector2f scanPos = Eigen::Rotation2Df(angle_diff) * scan_cloud_[i] + trans_diff;
    
            // Ignore scans that don't hit an object
            if(scanPos.norm() >= std::min(CONFIG_csm_eval_range_max, csm_map_.GetMaxRangeInScan())) {
              continue;
            }

            try{
              // Get log likelihood from correlative scan matching image
              double gaussian_prob = low_csm_map_.GetLogLikelihoodAtPosition(scanPos.x(), scanPos.y());
              pose_log_prob += CONFIG_csm_gamma*gaussian_prob;
              
            } catch(std::out_of_range) {
              continue;
            }
          }

          
          // Update most likely transform
          if(low_P < pose_log_prob){
            low_T = Pose2D<float>(angle_diff, trans_diff);
            low_P = pose_log_prob;
          }
          //std::cout << angle_diff << ", " << trans_diff.x() << ", " << trans_diff.y() << ", " << pose_log_prob << std::endl;
          // Add new search region to queue
          SearchRegion currRegion(angle_index, x_index, y_index, pose_log_prob);
          low_res_queue.push_back(currRegion);
        }
      }     
    }

    // Sort search regions by probability, where most likely regions will be evaluated with high-res map first
    std::sort(low_res_queue.begin(), low_res_queue.end(), CompareProb());

    double Li = 1;
    double H = -1e5;
    while(!low_res_queue.empty()){
      //std::cout << "Region Dump" << std::endl;
      
      SearchRegion currSearchRegion = low_res_queue.back();
      low_res_queue.pop_back();
      
      Li = currSearchRegion.prob;

      // Low resolution raster provides upper bound on max probability in a region
      // Search regions with the high res map until the high res likelihood is larger than all remaining search regions
      if(Li < H){
        break;
      }
      else{
        int high_res_start_theta_idx = (currSearchRegion.theta_index * CONFIG_low_theta_res - 0.5*CONFIG_low_theta_res) / (CONFIG_theta_res) - 1;
        int high_res_end_theta_idx = ((currSearchRegion.theta_index + 1) * CONFIG_low_theta_res + 0.5*CONFIG_low_theta_res) / (CONFIG_theta_res) + 1;
        int high_res_start_x_idx = (currSearchRegion.x_index * CONFIG_low_dist_res - 0.5*CONFIG_low_dist_res) / (CONFIG_dist_res) - 1;
        int high_res_end_x_idx = ((currSearchRegion.x_index + 1) * CONFIG_low_dist_res + 0.5*CONFIG_low_dist_res) / (CONFIG_dist_res) + 1;
        int high_res_start_y_idx = (currSearchRegion.y_index * CONFIG_low_dist_res - 0.5*CONFIG_low_dist_res) / (CONFIG_dist_res) - 1;
        int high_res_end_y_idx = ((currSearchRegion.y_index + 1) * CONFIG_low_dist_res + 0.5*CONFIG_low_dist_res) / (CONFIG_dist_res) + 1;
        
        for(int theta_index = high_res_start_theta_idx; theta_index <= high_res_end_theta_idx; theta_index++){
          float dtheta = theta_low + theta_index * CONFIG_theta_res;
          float angle_diff = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.angle + dtheta, CONFIG_theta_res);
          for(int x_index = high_res_start_x_idx; x_index <= high_res_end_x_idx; x_index++){
            float dx = translation_low + x_index * CONFIG_dist_res;
            Vector2f trans_diff;
            trans_diff[0] = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.translation.x() + dx, CONFIG_dist_res);

            for(int y_index = high_res_start_y_idx; y_index <= high_res_end_y_idx; y_index++){
              float dy = translation_low + y_index * CONFIG_dist_res;
              trans_diff[1] = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.translation.y() + dy, CONFIG_dist_res);
              
              double pose_log_prob = 0.0;
              
              // For each point in scan
              for(std::size_t i = 0; i < scan_cloud_.size(); i+=CONFIG_csm_resize){

                // Transform new point into frame of previous scan / reference image
                Vector2f scanPos = Eigen::Rotation2Df(angle_diff) * scan_cloud_[i] + trans_diff;

                // Ignore scans that don't hit an object
                if(scanPos.norm() >= std::min(CONFIG_csm_eval_range_max, csm_map_.GetMaxRangeInScan())) {
                  continue;
                }

                try{
                  // Get log likelihood from correlative scan matching image
                  double gaussian_prob = csm_map_.GetLogLikelihoodAtPosition(scanPos.x(), scanPos.y());
                  pose_log_prob += CONFIG_csm_gamma*gaussian_prob;

                } catch(std::out_of_range) {
                  continue;
                }
              }

              ComputeCovariance(trans_diff, angle_diff, pose_log_prob);
              
              // Update most likely transform
              if( H < pose_log_prob){
                H = pose_log_prob ;
                T = Pose2D<float>(angle_diff, trans_diff);
              }

              //std::cout << angle_diff << ", " << trans_diff.x() << ", " << trans_diff.y() << ", " << pose_log_prob << std::endl;
            }
          }
        }
      }
    }
    covariance = ComputeCovariance();
    //std::cout << std::endl << covariance << std::endl << std::endl;
    
    //std::cout << "X/Y/Theta STD: " << sqrt(covariance(0, 0)) << ", " << sqrt(covariance(1, 1)) << ", " << sqrt(covariance(2, 2)) << std::endl;

    // // Produces Image for most likely theta
    // float lr_angle_diff = low_T.angle;
    // // For each possible X
    // for(int x_index = 0; x_index <= (translation_high - translation_low) / CONFIG_low_dist_res; x_index++){
    //   float dx = translation_low + x_index * CONFIG_low_dist_res;
    //   Vector2f trans_diff;
    //   trans_diff[0] = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.translation.x() + dx, CONFIG_dist_res);
      
    //   // For each possible Y
    //   for(int y_index = 0; y_index <= (translation_high - translation_low) / CONFIG_low_dist_res; y_index++){
    //     float dy = translation_low + y_index * CONFIG_low_dist_res;
    //     trans_diff[1] = xy_raster_map::XYRasterMap::RoundToResolution(wheel_odom_.pose.translation.y() + dy, CONFIG_dist_res);
        
    //     double pose_log_prob = 0.0;

    //     // For each point in scan
    //     for(std::size_t i = 0; i < scan_cloud_.size(); i++){

    //       // Transform new point into frame of previous scan / reference image
    //       Vector2f scanPos = Eigen::Rotation2Df(lr_angle_diff) * scan_cloud_[i] + trans_diff;
  
    //       // Ignore scans that don't hit an object
    //       if(scanPos.norm() >= std::min(CONFIG_csm_eval_range_max, csm_map_.GetMaxRangeInScan())) {
    //         continue;
    //       }

    //       try{
    //         // Get log likelihood from correlative scan matching image
    //         double gaussian_prob = low_csm_map_.GetLogLikelihoodAtPosition(scanPos.x(), scanPos.y());
    //         pose_log_prob += CONFIG_csm_gamma*gaussian_prob;
    //       } catch(std::out_of_range) {
    //         continue;
    //       }
    //     }
    //     low_res_likelihood_cube_.SetTransformLikelihood(dx, dy, pose_log_prob);
    //   }
    // }
    // low_res_likelihood_cube_.DrawCSMImage();

    
    // Produces Image for most likely theta
    float angle_diff = T.angle;
    // For each possible X
    for(int x_index = 0; x_index <= (translation_high - translation_low) / CONFIG_dist_res; x_index++){
      float dx = translation_low + x_index * CONFIG_dist_res;
      Vector2f trans_diff;
      trans_diff[0] = csm_map_.RoundToResolution(wheel_odom_.pose.translation.x() + dx, CONFIG_dist_res);
      
      // For each possible Y
      for(int y_index = 0; y_index <= (translation_high - translation_low) / CONFIG_dist_res; y_index++){
        float dy = translation_low + y_index * CONFIG_dist_res;
        double pose_log_prob = 0.0;
        trans_diff[1] = csm_map_.RoundToResolution(wheel_odom_.pose.translation.y() + dy, CONFIG_dist_res);
        
        // For each point in scan
        for(std::size_t i = 0; i < scan_cloud_.size(); i+=CONFIG_csm_resize){

          // Transform new point into frame of previous scan / reference image
          Vector2f scanPos = Eigen::Rotation2Df(angle_diff) * scan_cloud_[i] + trans_diff;
  
          // Ignore scans that don't hit an object
          if(scanPos.norm() >= std::min(CONFIG_csm_eval_range_max, csm_map_.GetMaxRangeInScan())) {
            continue;
          }

          try{
            double gaussian_prob = csm_map_.GetLogLikelihoodAtPosition(scanPos.x(), scanPos.y());
            pose_log_prob += CONFIG_csm_gamma*gaussian_prob;
          } catch(std::out_of_range) {
            continue;
          }
        }

        likelihood_cube_.SetTransformLikelihood(dx, dy, pose_log_prob);
      }
    }

    likelihood_cube_.DrawCSMImage();

    Eigen::Matrix3f independent_covariance;
    independent_covariance << covariance(0, 0),                          0,                            0,
                              0,                          covariance(1, 1),                            0,
                              0,                                         0,             covariance(2, 2) + 0.03;

    lidar_odom_ = PoseWithCovariance(T, independent_covariance);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max,
                                  float angle_increment) {
                                    
  // Initialize cloud on first laser msg
  if(!csm_map_initialized){
    csm_map_ = csm_map::CSMMap(CONFIG_map_length_dist, CONFIG_dist_res, CONFIG_min_map_prob, CONFIG_range_max, CONFIG_csm_sigma_observation);
    low_csm_map_ = csm_map::CSMMap(CONFIG_map_length_dist, CONFIG_low_dist_res, CONFIG_min_map_prob, CONFIG_range_max, CONFIG_csm_sigma_observation);
    scan_cloud_ = vector<Vector2f>(ranges.size());

    ConvertScanToPointCloud(angle_min, angle_increment, ranges, scan_cloud_);
    csm_map_.GenerateMapFromNewScan(scan_cloud_);
    low_csm_map_.GenerateMapFromNewScan(scan_cloud_);

    low_csm_map_.DrawCSMImage();
    csm_map_.DrawCSMImage();
    csm_map_initialized = true;
  }

  // Calculate motion since last update
  double odom_dist = (wheel_odom_.pose.translation).norm();
  double delta_angle = wheel_odom_.pose.angle;

  // Update only if motion exceeds update threshold
  if((odom_dist > CONFIG_min_update_dist || std::abs(delta_angle) > CONFIG_min_update_angle) && std::strcmp(CONFIG_localization_mode.c_str(), "odom") != 0){
    low_csm_map_.DrawCSMImage();
    csm_map_.DrawCSMImage();
    static int i = 0;
    double start_time = GetMonotonicTime();

    // Get Lidar Odometry from Correlative Scan Matching
    auto rot_to_last_pose = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
    Vector2f odom_loc_diff = rot_to_last_pose*(prev_odom_loc_ - last_update_loc_);
    ConvertScanToPointCloud(angle_min, angle_increment, ranges, scan_cloud_);

    // Estimate Lidar Odometry for new laser scan
    double start = GetMonotonicTime();
    EstimateLidarOdometry(); // cloud_, csm_map_
    double end = GetMonotonicTime();

    //std::cout << "Exec Time: " << end - start << std::endl;
    // std::cout << "Wheel Odometry: " << wheel_odom_.pose.translation.x() << ", " << wheel_odom_.pose.translation.y() << ", " << wheel_odom_.pose.angle << std::endl;
    // std::cout << "Lidar Odometry: " << lidar_odom_.pose.translation.x() << ", " << lidar_odom_.pose.translation.y() << ", " << lidar_odom_.pose.angle << std::endl;

    if(std::strcmp(CONFIG_localization_mode.c_str(), "lidar") == 0){
      estimated_odom_.pose.ApplyPose(lidar_odom_.pose);
      estimated_odom_.covariance += lidar_odom_.covariance;
    }

    vector<Vector2f> tf_scan(ranges.size());
    vector<Vector2f> odom_scan(ranges.size());
    for(std::size_t i = 0; i < tf_scan.size(); i++){
      tf_scan[i] = Eigen::Rotation2D<float>(lidar_odom_.pose.angle)*scan_cloud_[i] + lidar_odom_.pose.translation;
      odom_scan[i] = Eigen::Rotation2D<float>(wheel_odom_.pose.angle)*scan_cloud_[i] + wheel_odom_.pose.translation;
    }
    
    //low_csm_map_.DrawScanCloudOnImage(tf_scan, CONFIG_csm_eval_range_max);
    csm_map_.DrawScanCloudOnImage(tf_scan, std::min(CONFIG_csm_eval_range_max, csm_map_.GetMaxRangeInScan()), false);
    csm_map_.DrawScanCloudOnImage(odom_scan, std::min(CONFIG_csm_eval_range_max, csm_map_.GetMaxRangeInScan()), true);
    

    // Update cost map with new cloud_
    csm_map_.GenerateMapFromNewScan(scan_cloud_);
    low_csm_map_.GenerateMapFromNewScan(scan_cloud_);

    if(std::strcmp(CONFIG_localization_mode.c_str(), "ekf") == 0){
      // EKF Update
      UpdateEKF();
    }

    if(std::strcmp(CONFIG_localization_mode.c_str(), "ekf_pf") == 0){
      // EKF Update
      UpdateEKF();

      for(Particle &particle: particles_){

        Eigen::Vector2f e_xy = Eigen::Vector2f((float) rng_.Gaussian(0.0, estimated_odom_.covariance(0,0)),(float) rng_.Gaussian(0.0, estimated_odom_.covariance(1,1)));
        Eigen::Vector2f noisy_translation = estimated_odom_.pose.translation + e_xy; // in previous base_link

        float noisy_angle = estimated_odom_.pose.angle + (float) rng_.Gaussian(0.0, estimated_odom_.covariance(2,2));
        
        // Transform noise to map using current particle angle
        auto rot_bl1_to_map = Eigen::Rotation2D<float>(particle.angle).toRotationMatrix();
        particle.loc += rot_bl1_to_map * noisy_translation;   
        particle.angle += noisy_angle;        
      }

      last_update_loc_ = prev_odom_loc_;
      last_update_angle_ = prev_odom_angle_;
      lidar_odom_ = PoseWithCovariance();
      wheel_odom_ = PoseWithCovariance();
      estimated_odom_ = PoseWithCovariance();

      // Initialize Particle Filter Update variables
      max_weight_log_ = -1e10;
      weight_sum_ = 0;
      double weight_sum_squared = 0;
      weight_bins_.resize(particles_.size());
      std::fill(weight_bins_.begin(), weight_bins_.end(), 0);

      // Update each particle with log error weight and find largest weight (smallest negative number)
      double particle_update_start = GetMonotonicTime();
      // double p_update_start = 0;
      // double p_update_diff_avg = 0;
      for(Particle &p: particles_){
        //p_update_start = GetMonotonicTime();

        Update(ranges, range_min, range_max, angle_min, angle_max, &p);
        max_weight_log_ = std::max(max_weight_log_, p.weight);
        
        //p_update_diff_avg += 1000000*(GetMonotonicTime() - p_update_start);
      }
      // Update loop profiling
      double particle_update_diff = 1000*(GetMonotonicTime() - particle_update_start);
      // p_update_diff_avg /= particles_.size();

      std::cout << "Update Time (ms): " << particle_update_diff << std::endl;

      for(std::size_t i = 0; i < particles_.size(); i++){
        // Normalize log weights by max, transform back to linear scale
        particles_[i].weight = exp(particles_[i].weight - max_weight_log_);
        
        // Sum all linear weights
        weight_sum_ += particles_[i].weight;
        weight_sum_squared += particles_[i].weight*particles_[i].weight;
        // Generate Resampling bins
        weight_bins_[i] = weight_sum_;
      }

      std::cout << "Effective Sampling Coefficient: " << 1/weight_sum_squared << std::endl << std::endl;

      // Resample
      if(!(resample_loop_counter_ % CONFIG_resample_frequency)){
        std::cout << "Resample" << std::endl;
        LowVarianceResample();
      }

      // Update previous state variables
      last_update_loc_ = prev_odom_loc_;
      last_update_angle_ = prev_odom_angle_;
      resample_loop_counter_++;

      end_time += 1000*(GetMonotonicTime() - start_time);
      
      // Profiling
      if(i%10 == 0){
        std::cout << "Total Update Avg (ms): " << end_time/10.0 << std::endl << std::endl;
        end_time = 0;
      }
      i++;
    }
  }                     
}



void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log.

  particles_.resize(CONFIG_num_particles);

  for(Particle &particle: particles_){
    particle.loc = Eigen::Vector2f(
      loc[0] + rng_.Gaussian(0, CONFIG_init_x_sigma),
      loc[1] + rng_.Gaussian(0, CONFIG_init_y_sigma)
      );
    particle.angle = angle + rng_.Gaussian(0, CONFIG_init_r_sigma);
    particle.weight = 1/((double)particles_.size());
  }
  max_weight_log_ = 0;
  last_update_loc_ = prev_odom_loc_;
  last_update_angle_ = prev_odom_angle_;

  estimated_odom_ = PoseWithCovariance();

  map_.Load(map_file);
  SortMap();
}

void ParticleFilter::SortMap(){
   // Split lines in map into horizontal, vertical, and angled
  horizontal_lines_.clear();
  vertical_lines_.clear();
  angled_lines_.clear();
  
  for (size_t i = 0; i < map_.lines.size(); ++i) {
      const geometry::line2f line = map_.lines[i];
      if(line.p0.y() == line.p1.y()){
        horizontal_lines_.push_back(line);
      }
      else if(line.p0.x() == line.p1.x()){
        vertical_lines_.push_back(line);
      }
      else{
        angled_lines_.push_back(line);
      }
  }
  // Sort horizontal and vertical in ascending order
  std::sort(horizontal_lines_.begin(), horizontal_lines_.end(), horizontal_line_compare);
  std::sort(vertical_lines_.begin(), vertical_lines_.end(), vertical_line_compare);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles.

  Eigen::Vector2f angle_point = Eigen::Vector2f(0, 0);
  for(Particle particle: particles_){
    loc += particle.loc * particle.weight;
    angle_point += Eigen::Vector2f(cos(particle.angle), sin(particle.angle)) * particle.weight;
  }

  loc /= weight_sum_;
  angle_point /= weight_sum_;
  angle = atan2(angle_point[1], angle_point[0]);
  Pose2D<float> curr_pose(angle, loc);
  // std::cout << "Get Location: " << curr_pose.translation.x() << ", " << curr_pose.translation.y() << ", " << curr_pose.angle << std::endl;
  // std::cout << "Estimated: " << estimated_odom_.pose.translation.x() << ", " << estimated_odom_.pose.translation.y() << ", " << estimated_odom_.pose.angle << std::endl;
  curr_pose.ApplyPose(estimated_odom_.pose);

  // Undo comment for temp particle filter motion localization
  loc = curr_pose.translation;
  angle = curr_pose.angle;
}

}  // namespace particle_filter
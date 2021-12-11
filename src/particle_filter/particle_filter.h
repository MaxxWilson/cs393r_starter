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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "shared/math/line2d.h"
#include "shared/math/poses_2d.h"
#include "shared/util/random.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "raster_map/csm_map.h"
#include "raster_map/transform_cube_slice.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

using pose_2d::Pose2D;

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

struct SearchRegion {
  int theta_index;
  int x_index;
  int y_index;
  double prob;
  
  SearchRegion(int theta_index, int x_index, int y_index, double prob):
  theta_index(theta_index), x_index(x_index), y_index(y_index), prob(prob){ 
  }
};

struct PoseWithCovariance {
  Pose2D<float> pose;
  Eigen::Matrix3f covariance;

  PoseWithCovariance(){
    pose = Pose2D<float>(0, Eigen::Vector2f(0, 0));
    covariance = Eigen::Matrix3f::Zero();
  }

  PoseWithCovariance(float rotation, Eigen::Vector2f translation, Eigen::Matrix3f covariance){
    this->pose = Pose2D<float>(rotation, translation);
    this->covariance = covariance;
  }

  PoseWithCovariance(Pose2D<float> pose, Eigen::Matrix3f covariance){
    this->pose = pose;
    this->covariance = covariance;
  }

  Eigen::Vector3f GetStateVector(){
    return Eigen::Vector3f(pose.translation.x(), pose.translation.y(), pose.angle);
  }

  void SetStateVector(Eigen::Vector3f new_state){
    this->pose = Pose2D<float>(new_state[2], Eigen::Vector2f(new_state[0],new_state[1]));
  }
};

struct CompareProb {
    bool operator()(struct SearchRegion const& p1, struct SearchRegion const& p2)
    {
        // return "true" if "p1" is ordered
        // before "p2", for example:
        return p1.prob < p2.prob;
    }
};


class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc, const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  void LowVarianceResample();

  void PredictEKF(const Eigen::Vector2f& odom_loc, const float odom_angle);

  void UpdateEKF();

  void SortMap();
  static bool horizontal_line_compare(const geometry::line2f l1, const geometry::line2f l2);
  static bool vertical_line_compare(const geometry::line2f l1, const geometry::line2f l2);

  void ConvertScanToPointCloud(const float angle_min, const float angle_increment, const std::vector<float>& ranges, std::vector<Eigen::Vector2f> &cloud);

  void EstimateLidarOdometry();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);

  void SetParticlesForTesting(std::vector<Particle> new_particles);
  cv::Mat GetTFCubeImage();
  cv::Mat GetLowResTFCubeImage();
  
  cv::Mat GetEKFProposalImage();
  cv::Mat GetLidarProposalImage();
  cv::Mat GetOdomProposalImage();

  Eigen::Vector2f BaseLinkToSensorFrame(const Eigen::Vector2f &loc, const float &angle);

  csm_map::CSMMap GetCSMMap();
  csm_map::CSMMap GetLRCSMMap();

 private:
  Eigen::Vector2f mean_odom_;
  Eigen::Vector2f mean_angle_point_;
  Eigen::Matrix3f K_;
  float s_;
  void InitializeCovariance();
  void ComputeCovariance(Eigen::Vector2f odom, float angle, float prob);
  Eigen::Matrix3f ComputeCovariance();

  PoseWithCovariance estimated_odom_;
  PoseWithCovariance wheel_odom_;
  PoseWithCovariance lidar_odom_;

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;
  std::vector<geometry::line2f> horizontal_lines_;
  std::vector<geometry::line2f> vertical_lines_;
  std::vector<geometry::line2f> angled_lines_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  //TODO: refactor with underscores
  //Uncertainty Matrix from Environment - updates when there is new odometry 
  Eigen::Matrix3f Q;
  //mean of odometry transform distribution 
  Eigen::Vector3f uq;
  transform_cube_slice::TransformCubeSlice odom_proposal_img;
  transform_cube_slice::TransformCubeSlice lidar_proposal_img;
  transform_cube_slice::TransformCubeSlice ekf_proposal_img;
  
  csm_map::CSMMap csm_map_;
  csm_map::CSMMap low_csm_map_;
  transform_cube_slice::TransformCubeSlice likelihood_cube_;
  transform_cube_slice::TransformCubeSlice low_res_likelihood_cube_;
  bool csm_map_initialized = false;
  std::vector<Eigen::Vector2f> scan_cloud_;

  std::vector<double> weight_bins_;
  double max_weight_log_ = 0;
  double weight_sum_ = 1.0;

  Eigen::Vector2f last_update_loc_;
  float last_update_angle_;
  int resample_loop_counter_ = 0;


  double end_time = 0;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_

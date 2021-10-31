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
\file    cost_map.cc
\brief   CostMap Implementation
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "cost_map.h"
#include "shared/math/statistics.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using math_util::Sq;
namespace costmap{

CONFIG_FLOAT(map_length_dist, "map_length_dist");
CONFIG_FLOAT(dist_update_thresh, "dist_update_thresh");
CONFIG_FLOAT(laser_offset, "laser_offset");
CONFIG_FLOAT(range_max, "range_max");
CONFIG_FLOAT(dist_res, "dist_res");
CONFIG_DOUBLE(sigma_observation, "sigma_observation");
CONFIG_DOUBLE(gamma, "gamma");
CONFIG_INT(row_num, "row_num");


CostMap::CostMap(): cost_map_vector(CONFIG_row_num, vector<double>(CONFIG_row_num, 0.0)){
    max_likelihood = 0.0;
}

// Given a scan, clear the lookup table and calculate new lookup table gaussian distribution values
void CostMap::UpdateMap(const vector<float>& ranges, float range_min,
                float range_max, float angle_min, float angle_max, float angle_increment){
    ClearMap();

    // For square kernel of odd size, length / 2 - 1, EX. 5x5 -> 2, 17x17 -> 8
    // Size Kernel based on Std of sensor measurement, where past 3 sigma probabilty falls off to zero
    int kernel_half_width = 3 * CONFIG_sigma_observation / CONFIG_dist_res;

    // Iterate through rays in scan
    for(std::size_t i = 0; i < ranges.size(); i++){
        if(ranges[i] < CONFIG_range_max){
            // Convert scans from polar to cartesian
            float angle = angle_min + angle_increment*i;
            float x = ranges[i]*cos(angle) + CONFIG_laser_offset;
            float y = ranges[i]*sin(angle);

            // Apply Gaussian Kernel to Scan point
            for(int row = -kernel_half_width; row <= kernel_half_width; row++){
                for(int col = -kernel_half_width; col <= kernel_half_width; col++){
                    // Get the position of bin corresponding to scan point
                    Eigen::Vector2f scan_point_bin(RoundToResolution(x), RoundToResolution(y));
                    
                    // Get current bin location based on scan point as reference
                    Eigen::Vector2f curr_position = scan_point_bin + Eigen::Vector2f(col, row) * CONFIG_dist_res;
                    
                    // Add bin likelihood using distance from mean
                    double dist_from_scan_point = (scan_point_bin - curr_position).norm();
                    // double log_likelihood = -Sq(dist_from_scan_point) / Sq(CONFIG_sigma_observation);

                    //Calculate normal gaussian distribution to be put into cost map
                    double gauss_dist = statistics::ProbabilityDensityGaussian(0.0, dist_from_scan_point, CONFIG_sigma_observation);

                    double current_likelihood = cost_map_vector[GetIndexFromDist(curr_position.x())] [GetIndexFromDist(curr_position.y())];
                    
                    // Find largest guassian likelihood for normalization
                    if(gauss_dist + current_likelihood > max_likelihood){
                        max_likelihood = gauss_dist + current_likelihood;
                    }
                    SetLikelihoodAtPosition(curr_position.x(), curr_position.y(), gauss_dist + current_likelihood);
                }   
            }
        }
    }
}

void CostMap::ClearMap(){
    for(std::size_t i = 0; i < cost_map_vector.size(); i++){
        std::fill(cost_map_vector[i].begin(), cost_map_vector[i].end(), 0.0);
    }
}

void CostMap::SetLikelihoodAtPosition(double x, double y, double log_likelihood){
    int xIdx = GetIndexFromDist(x);
    int yIdx = GetIndexFromDist(y);
    cost_map_vector[xIdx][yIdx] = log_likelihood;
}

double CostMap::GetLikelihoodAtPosition(double x, double y){
    int xIdx = GetIndexFromDist(x);
    int yIdx = GetIndexFromDist(y);
    return cost_map_vector[xIdx][yIdx] / max_likelihood; // returns normalized log likelihood
}

int CostMap::GetIndexFromDist(double dist){
    float dist_rounded = RoundToResolution(dist);
    float lower_bound = RoundToResolution(-(CONFIG_map_length_dist));
    return (int) ((dist_rounded - lower_bound) / CONFIG_dist_res);
}

float CostMap::RoundToResolution(float value){
    return ((float) ((int) (value/CONFIG_dist_res + 0.5*math_util::Sign(value))))*CONFIG_dist_res;
}

} // namespace CostMap
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
void CostMap::UpdateCostMap(const std::vector<Eigen::Vector2f> &cloud){
    ClearMap();

    // For square kernel of odd size, length / 2 - 1, EX. 5x5 -> 2, 17x17 -> 8
    // Size Kernel based on Std of sensor measurement, where past 3 sigma probabilty falls off to zero
    int kernel_half_width = 3 * CONFIG_sigma_observation / CONFIG_dist_res;

    // Iterate through rays in scan
    for(std::size_t i = 0; i < cloud.size(); i++){
        if(cloud[i].norm() > CONFIG_range_max){
            continue;
        }
        auto x = cloud[i].x();
        auto y = cloud[i].y();

        // Get the position of bin corresponding to scan point
        Eigen::Vector2f scan_point_bin(x, y);
        Eigen::Vector2f last_position(0, 0);

        // Apply Gaussian Kernel to Scan point
        for(int row = -kernel_half_width; row <= kernel_half_width; row++){
            for(int col = -kernel_half_width; col <= kernel_half_width; col++){
                // Get current bin location based on scan point as reference
                Eigen::Vector2f curr_position = scan_point_bin + Eigen::Vector2f(col, row) * CONFIG_dist_res;
                
                // Add bin likelihood using distance from mean
                double dist_from_scan_point = (scan_point_bin - curr_position).norm();
                // double log_likelihood = -Sq(dist_from_scan_point) / Sq(CONFIG_sigma_observation);

                //Calculate normal gaussian distribution to be put into cost map
                double gauss_dist = statistics::ProbabilityDensityGaussian(0.0, dist_from_scan_point, CONFIG_sigma_observation);
                try {
                    // double current_likelihood = cost_map_vector[GetIndexFromDist(curr_position.x())][GetIndexFromDist(curr_position.y())];
                    double current_likelihood = GetLikelihoodAtPosition(curr_position.x(), curr_position.y(), false);
                    // Find largest guassian likelihood for normalization
                    if(gauss_dist + current_likelihood > max_likelihood){
                        max_likelihood = gauss_dist + current_likelihood;
                    }
                    SetLikelihoodAtPosition(curr_position.x(), curr_position.y(), gauss_dist + current_likelihood);
                } catch(std::out_of_range) {
                    continue;
                }
            }   
        }
    }
}

void CostMap::DrawCostMap(CImg<float> &image){
    float red[3]= {1, 0, 0};
    float green[3]= {0, 1, 0};

    image.draw_circle(GetIndexFromDist(0), GetIndexFromDist(0), 3, green);

    for(int x = 0; x < CONFIG_row_num; x++){
        for(int y = 0; y < CONFIG_row_num; y++){
            // Image coordinate frame is LHR
            image.draw_point(x, CONFIG_row_num - y - 1, red, cost_map_vector[x][y] / max_likelihood);
        }
    }
}

void CostMap::DisplayImage(CImg<float> &image){
    CImgDisplay main_disp(image,"Click a point");
    while (!main_disp.is_closed()) {
      main_disp.wait();
      if (main_disp.button()) {
        const int x = main_disp.mouse_x();
        const int y = main_disp.mouse_y();
        if (x >=0 && y >=0 && x < image.width() && y < image.height()) {
          std::cout << "Value at " << x << "," << y << " : " << image(x, y) << std::endl;
        }
      }
    }
}

void CostMap::ClearMap(){
    for(std::size_t i = 0; i < cost_map_vector.size(); i++){
        std::fill(cost_map_vector[i].begin(), cost_map_vector[i].end(), 0.0);
    }
}

void CostMap::SetLikelihoodAtPosition(double x, double y, double likelihood){
    int xIdx = GetIndexFromDist(x);
    int yIdx = GetIndexFromDist(y);
    if(xIdx < 0 || xIdx >= cost_map_vector.size() || yIdx < 0 || yIdx >= cost_map_vector[0].size()) throw std::out_of_range("Out of boundaries");
    cost_map_vector[xIdx][yIdx] = likelihood;
}

double CostMap::GetLikelihoodAtPosition(double x, double y, bool normalized){
    int xIdx = GetIndexFromDist(x);
    int yIdx = GetIndexFromDist(y);
    if(xIdx < 0 || xIdx >= cost_map_vector.size() || yIdx < 0 || yIdx >= cost_map_vector[0].size()) throw std::out_of_range("Out of boundaries");
    if(normalized){
        return (cost_map_vector[xIdx][yIdx] / max_likelihood > 1e-4) ? cost_map_vector[xIdx][yIdx] / max_likelihood : 1e-5;
    }
    else{
        return cost_map_vector[xIdx][yIdx];
    }
}

int CostMap::GetIndexFromDist(double dist){
    float dist_rounded = RoundToResolution(dist, CONFIG_dist_res);
    float lower_bound = RoundToResolution(-(CONFIG_map_length_dist), CONFIG_dist_res);
    return (int) RoundToResolution((dist_rounded - lower_bound) / CONFIG_dist_res, 1.0); // integer round using CONFIG_dist_res/2
}

float CostMap::RoundToResolution(float value, float res){
    return ((float) ((int) (value/res + 0.5*math_util::Sign(value))))*res;
}

} // namespace CostMap
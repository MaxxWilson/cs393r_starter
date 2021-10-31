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

namespace costmap{

CONFIG_FLOAT(dist_update_thresh, "dist_update_thresh");
CONFIG_FLOAT(laser_offset, "laser_offset");
CONFIG_FLOAT(range_max, "range_max");
CONFIG_FLOAT(dist_res, "dist_res");
CONFIG_DOUBLE(sigma_observation, "sigma_observation");
CONFIG_DOUBLE(gamma, "gamma");
CONFIG_INT(row_num, "row_num");


CostMap::CostMap(): cost_map_vector(CONFIG_row_num, vector<double>(CONFIG_row_num, 0.0)){

}

void CostMap::UpdateMap(const vector<float>& ranges, float range_min,
                float range_max, float angle_min, float angle_max, float angle_increment){
    // Given a scan, clear the lookup table and calculate new lookup table

    float pdf_max = 3 * CONFIG_sigma_observation / CONFIG_dist_res;
    for(std::size_t i = 0; i < ranges.size(); i++){
        // Polar to Cartesian conversion
        float angle = angle_min + angle_increment*i;
        float x = ranges[i]*cos(angle) + 0.2;
        float y = ranges[i]*sin(angle);
        for(int row = 0; row < pdf_max * 2 + 1; row+= CONFIG_dist_res){
            for(int col = 0; col < pdf_max * 2 + 1; col+= CONFIG_dist_res){
                float log_liklihood = -math_util::Sq((Eigen::Vector2f(x,y) - Eigen::Vector2f(pdf_max + row, pdf_max + col)).norm()) / (2 * math_util::Sq(CONFIG_sigma_observation));
                SetLogLikelihoodAtPosition(x - pdf_max + row, y- pdf_max + col, log_liklihood);
            }   
        }
    }
}

void CostMap::SetLogLikelihoodAtPosition(double x, double y, float log_likelihood){
    int xIdx = GetIndexFromDist(x);
    int yIdx = GetIndexFromDist(y);
    cost_map_vector[xIdx][yIdx] = log_likelihood;
}

double CostMap::GetLogLikelihoodAtPosition(double x, double y){
    int xIdx = GetIndexFromDist(x);
    int yIdx = GetIndexFromDist(y);
    return cost_map_vector[xIdx][yIdx];
}

int CostMap::GetIndexFromDist(double dist){
    float dist_rounded = RoundToResolution(dist);
    float lower_bound = RoundToResolution(-(CONFIG_dist_update_thresh + CONFIG_range_max));
    return (int) ((dist_rounded - lower_bound) / CONFIG_dist_res);
}

float CostMap::RoundToResolution(float value){
    return ((float) ((int) (value/CONFIG_dist_res + 0.5*math_util::Sign(value))))*CONFIG_dist_res;
}

} // namespace CostMap
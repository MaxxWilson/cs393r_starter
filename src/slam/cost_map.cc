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
                float range_max, float angle_min, float angle_max){
    // Given a scan, clear the lookup table and calculate new lookup table

    float sum_dist; //summation of all pdfs

    // for(std::size_t i = 0; i < ranges.size(); i++){
    //     float pdf_at_landmark = math_util::Pow((-math_util::Sq(0 - ranges[i]) / (2 * math_util::Sq(CONFIG_sigma_observation))), (int) CONFIG_gamma);
    //     
    // }
    /*
        
    */
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
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
\file    xy_raster_map.cc
\brief   General 2D Probability Map With ROS Image interfacing
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include "raster_map/xy_raster_map.h"

namespace xy_raster_map{

    XYRasterMap::XYRasterMap(){

    }

    XYRasterMap::XYRasterMap(
        double map_half_dimension,
        double dist_res,
        double init_grid_val):
        map_length_dist(map_half_dimension),
        dist_res(dist_res),
        row_num(2*map_length_dist/dist_res + 1),
        init_grid_val(init_grid_val),
        max_likelihood(init_grid_val),
        prob_map(row_num, std::vector<double>(row_num, init_grid_val)){

        image = cv::Mat(row_num, row_num, CV_8UC3, cv::Scalar(255, 255, 255));
    }

    void XYRasterMap::ClearMap(){
        for(std::size_t i = 0; i < prob_map.size(); i++){
            std::fill(prob_map[i].begin(), prob_map[i].end(), init_grid_val);
        }
        max_likelihood = init_grid_val;
    }

    cv::Mat XYRasterMap::GetImage(){
        return image;
    }

    float XYRasterMap::RoundToResolution(const float value, const float res) const{
        return ((float) ((int) (value/res + 0.5*math_util::Sign(value))))*res;
    }

    int XYRasterMap::GetIndexFromDist(const double dist) const{
        float dist_rounded = RoundToResolution(dist, dist_res);
        float lower_bound = RoundToResolution(-(map_length_dist), dist_res);
        return (int) RoundToResolution((dist_rounded - lower_bound) / dist_res, 1.0); // integer round using dist_res/2
    }

    std::pair<int, int> XYRasterMap::GetIndexPairFromDist(const Eigen::Vector2f loc) const {
        int xIdx = GetIndexFromDist(loc.x());
        int yIdx = GetIndexFromDist(loc.y());
        return std::make_pair(xIdx, yIdx);
    }

    Eigen::Vector2f XYRasterMap::GetLocFromIndex(int xIdx, int yIdx) const {
        double x = -(map_length_dist) + xIdx * dist_res;
        double y = -(map_length_dist) + yIdx * dist_res;
        return Eigen::Vector2f(x, y);
    }
    

} // namespace xy_raster_map
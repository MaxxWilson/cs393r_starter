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
\file    transform_cube_slice.cc
\brief   Implements Likelihood Cube Display from Correlative Scan Matching
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include "shared/math/statistics.h"
#include "transform_cube_slice.h"

namespace transform_cube_slice {

    TransformCubeSlice::TransformCubeSlice() : xy_raster_map::XYRasterMap(){
    
    }

    TransformCubeSlice::TransformCubeSlice(
        double map_half_dimension,
        double dist_res,
        double init_grid_val) :
        xy_raster_map::XYRasterMap(map_half_dimension, dist_res, init_grid_val)
        {
    }

    void TransformCubeSlice::DrawCSMImage(){
        image = cv::Mat(row_num, row_num, CV_8UC3, cv::Scalar(255, 255, 255));
        for(int x = 0; x < row_num; x++){
            for(int y = 0; y < row_num; y++){
                // Image coordinate frame is LHR, BGR
                int image_y = row_num - y - 1;
                image.at<cv::Vec3b>(image_y, x)[0] = 255 - (int) (255*(ConvertLogToStandard(prob_map[x][y])/ConvertLogToStandard(max_likelihood)));
                image.at<cv::Vec3b>(image_y, x)[1] = 255 - (int) (255*(ConvertLogToStandard(prob_map[x][y])/ConvertLogToStandard(max_likelihood)));
                image.at<cv::Vec3b>(image_y, x)[2] = 255;
            }
        }

        double origin = GetIndexFromDist(0.0);

        for(int x = origin - 1; x <= origin + 1; x++){
            for(int y = origin - 1; y <= origin + 1; y++){
                image.at<cv::Vec3b>(y, x)[0] = 0;
                image.at<cv::Vec3b>(y, x)[1] = 0;
                image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    void TransformCubeSlice::SetLogLikelihoodAtPosition(double x, double y, double likelihood){
        uint64_t xIdx = GetIndexFromDist(x);
        uint64_t yIdx = GetIndexFromDist(y);
        if(xIdx >= prob_map.size() || yIdx >= prob_map[0].size()) throw std::out_of_range("Out of boundaries");
        prob_map[xIdx][yIdx] = likelihood;
    }

    double TransformCubeSlice::GetLogLikelihoodAtPosition(double x, double y) const{
        uint64_t xIdx = GetIndexFromDist(x);
        uint64_t yIdx = GetIndexFromDist(y);
        if(xIdx >= prob_map.size() || yIdx >= prob_map[0].size()) throw std::out_of_range("Out of boundaries");
        return prob_map[xIdx][yIdx];
    }

    double TransformCubeSlice::GetNormalizedLikelihoodAtPosition(double x, double y) const{
        return GetStandardLikelihoodAtPosition(x, y)/ConvertLogToStandard(max_likelihood);
    }

    double TransformCubeSlice::GetStandardLikelihoodAtPosition(double x, double y) const{
        return ConvertLogToStandard(GetLogLikelihoodAtPosition(x, y));
    }

    double TransformCubeSlice::ConvertLogToStandard(double log_likelihood) const{
        return 1/(sigma_observation*sqrt(2*M_PI))*exp(log_likelihood);
    }
} // namespace TransformCubeSlice
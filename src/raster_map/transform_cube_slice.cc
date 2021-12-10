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

    void TransformCubeSlice::SetTransformLikelihood(const double x, const double y, const double val){
        int xIdx = GetIndexFromDist(x);
        int yIdx = GetIndexFromDist(y);
        prob_map[xIdx][yIdx] = val;
        max_likelihood = std::max(max_likelihood, val);
    }

    void TransformCubeSlice::DrawCSMImage(){
        image = cv::Mat(2*row_num, 2*row_num, CV_8UC3, cv::Scalar(255, 255, 255));
        for(int x = 0; x < row_num; x++){
            for(int y = 0; y < row_num; y++){
                // Image coordinate frame is LHR, BGR
                int image_y = row_num - y - 1;
                for(int px = -1; px <= 0; px++){
                    for(int py = -1; py <= 0; py++){
                        int xd = std::min(std::max(2*x + px, 0), 2*row_num - 1);
                        int yd = std::min(std::max(2*image_y + py, 0), 2*row_num - 1);
                        
                        image.at<cv::Vec3b>(yd, xd)[0] = 255 - (int) (255*exp(prob_map[x][y] - max_likelihood));
                        image.at<cv::Vec3b>(yd, xd)[1] = 255 - (int) (255*exp(prob_map[x][y] - max_likelihood));
                        image.at<cv::Vec3b>(yd, xd)[2] = 255;
                    }
                }

            }
        }
    }
} // namespace TransformCubeSlice
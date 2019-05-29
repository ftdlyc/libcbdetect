// c++ version by ftdlyc

/*
% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#pragma once
#ifndef LIBCBDETECT_BOX_FILTER_H
#define LIBCBDETECT_BOX_FILTER_H

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void box_filter(const cv::Mat& img, cv::Mat& blur_img, int kernel_size_x, int kernel_size_y = -1);

LIBCBDETECT_DLL_DECL void image_normalization_and_gradients(cv::Mat& img, cv::Mat& img_du, cv::Mat& img_dv,
                                                            cv::Mat& img_angle, cv::Mat& img_weight,
                                                            const Params& params);

} // namespace cbdetect

#endif //LIBCBDETECT_BOX_FILTER_H

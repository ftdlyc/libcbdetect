/**
* Copyright 2018, ftdlyc <yclu.cn@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#ifndef LIBCBDETECT_GET_IMAGE_PATCH_H
#define LIBCBDETECT_GET_IMAGE_PATCH_H

#include <opencv2/opencv.hpp>

namespace cbdetect {

void get_image_patch(const cv::Mat& img, double u, double v, int r, cv::Mat& img_sub);

void get_image_patch_with_mask(const cv::Mat& img, const cv::Mat& mask, double u, double v, int r, cv::Mat& img_sub);

} // namespace cbdetect

#endif //LIBCBDETECT_GET_IMAGE_PATCH_H

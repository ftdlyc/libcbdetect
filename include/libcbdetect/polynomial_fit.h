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
#ifndef LIBCBDETECT_POLYNOMIAL_FIT_H
#define LIBCBDETECT_POLYNOMIAL_FIT_H

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"

namespace cbdetect {

void polynomial_fit(const cv::Mat& img, Corner& corners, const Params& params);

}

#endif //LIBCBDETECT_POLYNOMIAL_FIT_H

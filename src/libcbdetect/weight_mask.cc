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

#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"
#include "libcbdetect/weight_mask.h"

namespace cbdetect {

std::unordered_map<int, cv::Mat> weight_mask(const std::vector<int>& radius) {
  std::unordered_map<int, cv::Mat> mask;
  for(const auto& r : radius) {
    mask[r]      = cv::Mat::zeros(r * 2 + 1, r * 2 + 1, CV_64F);
    cv::Mat& mat = mask[r];
    for(int v = 0; v < r * 2 + 1; ++v) {
      for(int u = 0; u < r * 2 + 1; ++u) {
        double dist          = std::sqrt((u - r) * (u - r) + (v - r) * (v - r)) / r;
        dist                 = std::min(std::max(dist, 0.7), 1.3);
        mat.at<double>(v, u) = (1.3 - dist) / 0.6;
      }
    }
  }
  return mask;
};

} // namespace cbdetect
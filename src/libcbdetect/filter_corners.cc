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

#include "filter_corners.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "find_corners.h"
#include "find_modes_meanshift.h"
#include "weight_mask.h"

namespace cbdetect {

void filter_corners(const cv::Mat &img, const cv::Mat &img_angle, const cv::Mat &img_weight,
                    const std::vector<int> &radius, Corner &corners) {
  int n = 32, width = img.cols, height = img.rows;
  std::vector<cv::Point2d> corners_out_p;
  std::vector<int> corners_out_r;
  std::vector<double> cos_v(n), sin_v(n);
  for (int i = 0; i < n; ++i) {
    cos_v[i] = std::cos(i * 2.0 * M_PI / (n - 1));
    sin_v[i] = std::sin(i * 2.0 * M_PI / (n - 1));
  }
  auto mask = weight_mask(radius);

  for (int i = 0; i < corners.p.size(); ++i) {
    int num_crossings = 0, num_modes = 0;
    int center_u = std::round(corners.p[i].x);
    int center_v = std::round(corners.p[i].y);
    int r = corners.r[i];

    // extract circle locations and its value
    std::vector<double> c(n);
    for (int j = 0; j < n; ++j) {
      int circle_u = static_cast<int>(std::round(center_u + 0.75 * r * cos_v[j]));
      int circle_v = static_cast<int>(std::round(center_v + 0.75 * r * sin_v[j]));
      circle_u = std::min(std::max(circle_u, 0), width - 1);
      circle_v = std::min(std::max(circle_v, 0), height - 1);
      c[j] = img.at<double>(circle_v, circle_u);
    }
    auto minmax = std::minmax_element(c.begin(), c.end());
    double min_c = *minmax.first, max_c = *minmax.second;
    for (int j = 0; j < n; ++j) {
      c[j] = c[j] - min_c - (max_c - min_c) / 2;
    }

    // count number of zero-crossings
    int fisrt_cross_index = 0;
    for (int j = 0; j < n; ++j) {
      if ((c[j] > 0) ^ (c[(j + 1) % n] > 0)) {
        fisrt_cross_index = (j + 1) % n;
        break;
      }
    }
    for (int j = fisrt_cross_index, count = 0; j < n + fisrt_cross_index; ++j, ++count) {
      if ((c[j % n] > 0) ^ (c[(j + 1) % n] > 0)) {
        if (count >= n / 8) { ++num_crossings; }
        count = 0;
      }
    }

    int top_left_u = std::max(center_u - r, 0);
    int top_left_v = std::max(center_v - r, 0);
    int bottom_right_u = std::min(center_u + r, width - 1);
    int bottom_right_v = std::min(center_v + r, height - 1);
    cv::Mat img_weight_sub = cv::Mat::zeros(2 * r + 1, 2 * r + 1, CV_64F);
    img_weight.rowRange(top_left_v, bottom_right_v + 1).colRange(top_left_u, bottom_right_u + 1).copyTo(
        img_weight_sub(cv::Range(top_left_v - center_v + r, bottom_right_v - center_v + r + 1),
                       cv::Range(top_left_u - center_u + r, bottom_right_u - center_u + r + 1)));
    img_weight_sub = img_weight_sub.mul(mask[r]);
    double tmp_maxval = 0;
    cv::minMaxLoc(img_weight_sub, NULL, &tmp_maxval);
    img_weight_sub.forEach<double>([&tmp_maxval](double &val, const int *pos) -> void {
      val = val < 0.5 * tmp_maxval ? 0 : val;
    });

    // create histogram
    std::vector<double> angle_hist(n, 0);
    for (int j2 = top_left_v; j2 <= bottom_right_v; ++j2) {
      for (int i2 = top_left_u; i2 <= bottom_right_u; ++i2) {
        int bin = static_cast<int>(std::floor(img_angle.at<double>(j2, i2) / (M_PI / n)));
        angle_hist[bin] += img_weight_sub.at<double>(j2 - center_v + r, i2 - center_u + r);
      }
    }

    auto modes = find_modes_meanshift(angle_hist, 1.5);
    for (const auto &j : modes) {
      if (2 * j.second > modes[0].second) { ++num_modes; }
    }

    if (num_crossings == 4 && num_modes == 2) {
      corners_out_p.emplace_back(cv::Point2d(center_u, center_v));
      corners_out_r.emplace_back(r);
    }
  }
  corners.p = std::move(corners_out_p);
  corners.r = std::move(corners_out_r);
}

}

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

#include "image_normalization_and_gradients.h"
#include <cmath>
#include <opencv2/core/hal/hal.hpp>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

void box_filter(const cv::Mat &img, cv::Mat &blur_img, int kernel_size) {
  blur_img.create(img.size(), CV_64F);
  std::vector<double> buf(img.cols, 0);
  std::vector<int> count_buf(img.cols, 0);
  int count = 0;
  for (int j = 0; j < std::min(kernel_size, img.rows - 1); ++j) {
    for (int i = 0; i < img.cols; ++i) {
      buf[i] += img.at<double>(j, i);
      ++count_buf[i];
    }
  }
  for (int j = 0; j < img.rows; ++j) {
    if (j > kernel_size) {
      for (int i = 0; i < img.cols; ++i) {
        buf[i] -= img.at<double>(j - kernel_size - 1, i);
        --count_buf[i];
      }
    }
    if (j + kernel_size < img.rows) {
      for (int i = 0; i < img.cols; ++i) {
        buf[i] += img.at<double>(j + kernel_size, i);
        ++count_buf[i];
      }
    }
    blur_img.at<double>(j, 0) = 0;
    count = 0;
    for (int i = 0; i <= std::min(kernel_size, img.cols - 1); ++i) {
      blur_img.at<double>(j, 0) += buf[i];
      count += count_buf[i];
    }
    for (int i = 1; i < img.cols; ++i) {
      blur_img.at<double>(j, i) = blur_img.at<double>(j, i - 1);
      blur_img.at<double>(j, i - 1) /= count;
      if (i > kernel_size) {
        blur_img.at<double>(j, i) -= buf[i - kernel_size - 1];
        count -= count_buf[i - kernel_size - 1];
      }
      if (i + kernel_size < img.cols) {
        blur_img.at<double>(j, i) += buf[i + kernel_size];
        count += count_buf[i + kernel_size];
      }
    }
    blur_img.at<double>(j, img.cols - 1) /= count;
  }
}

void image_normalization_and_gradients(cv::Mat &img, cv::Mat &img_du, cv::Mat &img_dv,
                                       cv::Mat &img_angle, cv::Mat &img_weight, const Params &params) {
  // normalize image
  if (params.norm) {
    cv::Mat blur_img;
    box_filter(img, blur_img, params.norm_half_kernel_size);
    img = img - blur_img;
    img = 2.5 * (cv::max(cv::min(img + 0.2, 0.4), 0));
  }

  // sobel masks
  cv::Mat_<double> du({3, 3}, {1, 0, -1, 2, 0, -2, 1, 0, -1});
  cv::Mat_<double> dv({3, 3}, {1, 2, 1, 0, 0, 0, -1, -2, -1});

  // compute image derivatives (for principal axes estimation)
  cv::filter2D(img, img_du, -1, du, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
  cv::filter2D(img, img_dv, -1, dv, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
  img_angle.create(img.size(), img.type());
  img_weight.create(img.size(), img.type());
  if (!img_du.isContinuous()) {
    cv::Mat tmp = img_du.clone();
    std::swap(tmp, img_du);
  }
  if (!img_dv.isContinuous()) {
    cv::Mat tmp = img_dv.clone();
    std::swap(tmp, img_dv);
  }
  if (!img_angle.isContinuous()) {
    cv::Mat tmp = img_angle.clone();
    std::swap(tmp, img_angle);
  }
  if (!img_weight.isContinuous()) {
    cv::Mat tmp = img_weight.clone();
    std::swap(tmp, img_weight);
  }
  auto img_du_data = (double *) img_du.data;
  auto img_dv_data = (double *) img_dv.data;
  auto img_angle_data = (double *) img_angle.data;
  auto img_weight_data = (double *) img_weight.data;
  cv::hal::fastAtan64f((const double *) img_dv.data, (const double *) img_du.data,
                       (double *) img_angle.data, img.rows * img.cols, false);
  for (int i = 0; i < img.rows * img.cols; ++i) {
    // correct angle to lie in between [0, M_PI]
    img_angle_data[i] = img_angle_data[i] >= M_PI ? img_angle_data[i] - M_PI : img_angle_data[i];
  }
  for (int i = 0; i < img.rows * img.cols; ++i) {
    img_weight_data[i] = std::sqrt(img_du_data[i] * img_du_data[i] + img_dv_data[i] * img_dv_data[i]);
  }

  // scale input image
  double img_min = 0, img_max = 1;
  cv::minMaxLoc(img, &img_min, &img_max);
  img = (img - img_min) / (img_max - img_min);
}

}

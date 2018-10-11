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

#include "find_corners.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "create_correlation_patch.h"
#include "filter_corners.h"
#include "image_normalization_and_gradients.h"
#include "non_maximum_suppression.h"
#include "refine_corners.h"
#include "score_corners.h"

namespace cbdetect {

void find_corners_in_image(const cv::Mat &img, Corner &corners, const Params &params) {
  // convert to double grayscale image
  cv::Mat img_norm;
  img.convertTo(img_norm, CV_64F, 1 / 255.0, 0);

  // normalize image and calculate gradients
  cv::Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight);

  // templates and scales
  std::vector<double> tprops;
  if (params.fast_mode) {
    tprops = {0, M_PI_2,
              M_PI_4, -M_PI_4};
  } else {
    tprops = {0, M_PI_2,
              M_PI_4, -M_PI_4,
              0, M_PI_4,
              0, -M_PI_4,
              M_PI_4, M_PI_2,
              -M_PI_4, M_PI_2,
              -3 * M_PI / 8, 3 * M_PI / 8,
              -M_PI / 8, M_PI / 8,
              -M_PI / 8, -3 * M_PI / 8,
              M_PI / 8, 3 * M_PI / 8};
  }
  std::vector<int> radius{4, 6, 10};

  // for all scales do
  corners.p.clear();
  corners.r.clear();
  corners.v1.clear();
  corners.v2.clear();
  corners.score.clear();
  for (const auto &r : radius) {
    // filter image
    cv::Mat img_corners = cv::Mat::zeros(img_norm.size(), CV_64F);
    cv::Mat img_corners_a1, img_corners_a2, img_corners_b1, img_corners_b2, img_corners_mu,
        img_corners_a, img_corners_b, img_corners_s1, img_corners_s2;

    for (int i = 0; i < tprops.size(); i += 2) {
      std::vector<cv::Mat> template_kernel(4); // a1, a2, b1, b2
      create_correlation_patch(template_kernel, tprops[i], tprops[i + 1], r);

      // filter image with current template
      cv::filter2D(img_norm, img_corners_a1, -1, template_kernel[0], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      cv::filter2D(img_norm, img_corners_a2, -1, template_kernel[1], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      cv::filter2D(img_norm, img_corners_b1, -1, template_kernel[2], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      cv::filter2D(img_norm, img_corners_b2, -1, template_kernel[3], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

      // compute mean
      img_corners_mu = (img_corners_a1 + img_corners_a2 + img_corners_b1 + img_corners_b2) / 4;

      // case 1: a=white, b=black
      img_corners_a = cv::min(img_corners_a1, img_corners_a2) - img_corners_mu;
      img_corners_b = img_corners_mu - cv::max(img_corners_b1, img_corners_b2);
      img_corners_s1 = cv::min(img_corners_a, img_corners_b);
      // case 2: b=white, a=black
      img_corners_a = img_corners_mu - cv::max(img_corners_a1, img_corners_a2);
      img_corners_b = cv::min(img_corners_b1, img_corners_b2) - img_corners_mu;
      img_corners_s2 = cv::min(img_corners_a, img_corners_b);

      // combine both
      img_corners = cv::max(img_corners, cv::max(img_corners_s1, img_corners_s2));
    }
    non_maximum_suppression(img_corners, 1, params.tau, r, corners);
  }

  if (corners.p.empty()) { return; }
  if (params.show_processing) {
    printf("Filtering image (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, radius, corners);
  if (params.show_processing) {
    printf("Filtering corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }

  // subpixel refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, radius, corners);
  if (params.show_processing) {
    printf("Refining corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }

  // score corners
  sorce_corners(img_norm, img_weight, radius, corners);

  // remove low scoring corners
  remove_low_scoring_corners(params.tau, corners);

  // non maximum suppression
  non_maximum_suppression_sparse(corners, 3, img.size());
  if (params.show_processing) {
    printf("Scoring corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }
}

void find_corners(const cv::Mat &img, Corner &corners, const Params &params) {
  // image type check
  cv::Mat img_grey;
  if (img.channels() == 3) {
    cv::cvtColor(img, img_grey, CV_BGRA2GRAY);
  } else {
    img_grey = img.clone();
  }

  cv::Mat img_resized;
  cv::resize(img_grey, img_resized, cv::Size(img_grey.cols / 2, img_grey.rows / 2), 0, 0, cv::INTER_LINEAR);
  Corner corners_1, corners_2;
  find_corners_in_image(img_grey, corners_1, params);
  find_corners_in_image(img_resized, corners_2, params);
  std::for_each(corners_2.p.begin(), corners_2.p.end(), [](auto &p) { p *= 2; });

  int n_1 = corners_1.p.size();
  corners.p = std::move(corners_1.p);
  corners.r = std::move(corners_1.r);
  corners.v1 = std::move(corners_1.v1);
  corners.v2 = std::move(corners_1.v2);
  corners.score = std::move(corners_1.score);
  for (int i = 0; i < corners_2.p.size(); ++i) {
    double min_dist = 1e10;
    cv::Point2d &p2 = corners_2.p[i];
    for (int j = 0; j < n_1; ++j) {
      cv::Point2d &p1 = corners.p[j];
      double dist = cv::norm(p2 - p1);
      min_dist = dist < min_dist ? dist : min_dist;
    }
    if (min_dist > 5) {
      corners.p.emplace_back(corners_2.p[i]);
      corners.r.emplace_back(corners_2.r[i]);
      corners.v1.emplace_back(corners_2.v1[i]);
      corners.v2.emplace_back(corners_2.v2[i]);
      corners.score.emplace_back(corners_2.score[i]);
    }
  }

  if (params.show_processing) { printf("Total find corners ... %lu\n", corners.p.size()); }
}

}

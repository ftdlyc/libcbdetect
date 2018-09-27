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

#include "score_corners.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "create_correlation_patch.h"
#include "find_corners.h"
#include "weight_mask.h"

namespace cbdetect {

double corner_correlation_score(const cv::Mat &img, const cv::Mat &img_weight,
                                const cv::Point2d &v1, const cv::Point2d &v2) {
  // compute gradient filter kernel (bandwith = 3 px)
  double center = (img.cols - 1) / 2;
  cv::Mat img_filter = cv::Mat::ones(img.size(), CV_64F) * -1;
  for (int u = 0; u < img.cols; ++u) {
    for (int v = 0; v < img.rows; ++v) {
      cv::Point2d p1{u - center, v - center};
      cv::Point2d p2{(p1.x * v1.x + p1.y * v1.y) * v1.x, (p1.x * v1.x + p1.y * v1.y) * v1.y};
      cv::Point2d p3{(p1.x * v2.x + p1.y * v2.y) * v2.x, (p1.x * v2.x + p1.y * v2.y) * v2.y};
      if (cv::norm(p1 - p2) <= 1.5 || cv::norm(p1 - p3) <= 1.5) {
        img_filter.at<double>(v, u) = 1;
      }
    }
  }

  // normalize
  cv::Scalar mean, std;
  cv::meanStdDev(img_filter, mean, std);
  img_filter = (img_filter - mean[0]) / std[0];
  cv::meanStdDev(img_weight, mean, std);
  cv::Mat img_weight_norm = (img_weight - mean[0]) / std[0];

  // compute gradient score
  double score_gradient = cv::sum(img_weight_norm.mul(img_filter))[0];
  score_gradient = std::max(score_gradient / (img.cols * img.rows - 1), 0.);

  // create intensity filter kernel
  std::vector<cv::Mat> template_kernel(4); // a1, a2, b1, b2
  create_correlation_patch(template_kernel, std::atan2(v1.y, v1.x), std::atan2(v2.y, v2.x), (img.cols - 1) / 2);

  // checkerboard responses
  double a1 = cv::sum(img.mul(template_kernel[0]))[0];
  double a2 = cv::sum(img.mul(template_kernel[1]))[0];
  double b1 = cv::sum(img.mul(template_kernel[2]))[0];
  double b2 = cv::sum(img.mul(template_kernel[3]))[0];

  // mean
  double mu = (a1 + a2 + b1 + b2) / 4;

  // case 1: a=white, b=black
  double s1 = std::min(std::min(a1, a2) - mu, mu - std::min(b1, b2));

  // case 2: b=white, a=black
  double s2 = std::min(mu - std::min(a1, a2), std::min(b1, b2) - mu);

  // intensity score: max. of the 2 cases
  double score_intensity = std::max(std::max(s1, s2), 0.);

  // final score: product of gradient and intensity score
  return score_gradient * score_intensity;
}

void sorce_corners(const cv::Mat &img, const cv::Mat &img_weight, const std::vector<int> &radius, Corner &corners) {
  corners.score.resize(corners.p.size());
  int width = img.cols, height = img.rows;
  auto mask = weight_mask(radius);

  // for all corners do
  for (int i = 0; i < corners.p.size(); ++i) {
    // corner location
    int u = std::round(corners.p[i].x);
    int v = std::round(corners.p[i].y);
    int r = corners.r[i];

    if (u - r < 0 || u + r >= width || v - r < 0 || v + r >= height) {
      corners.score[i] = 0.;
      continue;
    }
    cv::Mat img_sub = img.rowRange(v - r, v + r + 1).colRange(u - r, u + r + 1).clone();
    cv::Mat img_weight_sub = img_weight.rowRange(v - r, v + r + 1).colRange(u - r, u + r + 1).clone();
    img_weight_sub = img_weight_sub.mul(mask[r]);
    corners.score[i] = corner_correlation_score(img_sub, img_weight_sub, corners.v1[i], corners.v2[i]);
  }
}

void remove_low_scoring_corners(double tau, Corner &corners) {
  std::vector<cv::Point2d> corners_out_p, corners_out_v1, corners_out_v2;
  std::vector<double> corners_out_score;
  std::vector<int> corners_out_r;
  for (int i = 0; i < corners.p.size(); ++i) {
    if (corners.score[i] > tau) {
      corners_out_p.emplace_back(corners.p[i]);
      corners_out_r.emplace_back(corners.r[i]);
      corners_out_v1.emplace_back(corners.v1[i]);
      corners_out_v2.emplace_back(corners.v2[i]);
      corners_out_score.emplace_back(corners.score[i]);
    }
  }
  corners.p = std::move(corners_out_p);
  corners.r = std::move(corners_out_r);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
  corners.score = std::move(corners_out_score);
}

}

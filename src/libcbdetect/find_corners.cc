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

#include <math.h>
#include <stdio.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"
#include "libcbdetect/filter_corners.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/get_init_location.h"
#include "libcbdetect/image_normalization_and_gradients.h"
#include "libcbdetect/non_maximum_suppression.h"
#include "libcbdetect/plot_corners.h"
#include "libcbdetect/polynomial_fit.h"
#include "libcbdetect/refine_corners.h"
#include "libcbdetect/score_corners.h"

namespace cbdetect {

void find_corners_reiszed(const cv::Mat& img, Corner& corners, const Params& params) {
  cv::Mat img_resized, img_norm;
  Corner corners_resized;

  // resize image
  double scale = 0;
  if(img.rows < 640 || img.cols < 480) {
    scale = 2.0;
  } else if(img.rows >= 640 || img.cols >= 480) {
    scale = 0.5;
  } else {
    return;
  }
  cv::resize(img, img_resized, cv::Size(img.cols * scale, img.rows * scale), 0, 0, cv::INTER_LINEAR);

  if(img_resized.channels() == 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img_resized, img_norm, cv::COLOR_BGR2GRAY);
#else
    cv::cvtColor(img_resized, img_norm, CV_BGR2GRAY);
#endif
    img_norm.convertTo(img_norm, CV_64F, 1 / 255.0, 0);
  } else {
    img_resized.convertTo(img_norm, CV_64F, 1 / 255.0, 0);
  }

  // normalize image and calculate gradients
  cv::Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);
  if(params.show_debug_image && params.norm) {
    cv::Mat img_show;
    img_norm.convertTo(img_show, CV_8U, 255, 0);
    cv::imshow("norm image resized", img_show);
    cv::waitKey();
  }

  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners_resized, params);
  if(corners_resized.p.empty()) {
    return;
  }
  if(params.show_processing) {
    printf("Initializing conres (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners_resized.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img_resized, corners_resized.p, "init location resized");
  }

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners_resized, params);
  if(params.show_processing) {
    printf("Filtering corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners_resized.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img_resized, corners_resized.p, "filter corners resized");
  }

  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners_resized, params);
  if(params.show_processing) {
    printf("Refining corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners_resized.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img_resized, corners_resized.p, "refine corners resized");
  }

  // merge corners
  std::for_each(corners_resized.p.begin(), corners_resized.p.end(), [&scale](auto& p) { p /= scale; });
  // std::for_each(corners_resized.r.begin(), corners_resized.r.end(), [&scale](auto &r) { r = (double) r / scale; });
  double min_dist_thr = scale > 1 ? 3 : 5;
  for(int i = 0; i < corners_resized.p.size(); ++i) {
    double min_dist = DBL_MAX;
    cv::Point2d& p2 = corners_resized.p[i];
    for(int j = 0; j < corners.p.size(); ++j) {
      cv::Point2d& p1 = corners.p[j];
      double dist     = cv::norm(p2 - p1);
      min_dist        = dist < min_dist ? dist : min_dist;
    }
    if(min_dist > min_dist_thr) {
      corners.p.emplace_back(corners_resized.p[i]);
      corners.r.emplace_back(corners_resized.r[i]);
      corners.v1.emplace_back(corners_resized.v1[i]);
      corners.v2.emplace_back(corners_resized.v2[i]);
      if(params.corner_type == MonkeySaddlePoint) {
        corners.v3.emplace_back(corners_resized.v3[i]);
      }
    }
  }
}

void find_corners(const cv::Mat& img, Corner& corners, const Params& params) {
  // clear old data
  corners.p.clear();
  corners.r.clear();
  corners.v1.clear();
  corners.v2.clear();
  corners.v3.clear();
  corners.score.clear();

  // convert to double grayscale image
  cv::Mat img_norm;
  if(img.channels() == 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_norm, cv::COLOR_BGR2GRAY);
#else
    cv::cvtColor(img, img_norm, CV_BGR2GRAY);
#endif
    img_norm.convertTo(img_norm, CV_64F, 1. / 255., 0);
  } else {
    img.convertTo(img_norm, CV_64F, 1. / 255., 0);
  }

  // normalize image and calculate gradients
  cv::Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);
  if(params.show_debug_image && params.norm) {
    cv::Mat img_show;
    img_norm.convertTo(img_show, CV_8U, 255., 0);
    cv::imshow("norm image", img_show);
    cv::waitKey();
  }

  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners, params);
  if(corners.p.empty()) {
    return;
  }
  if(params.show_processing) {
    printf("Initializing conres (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img, corners.p, "init location");
  }

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners, params);
  if(params.show_processing) {
    printf("Filtering corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img, corners.p, "filter corners");
  }

  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners, params);
  if(params.show_processing) {
    printf("Refining corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img, corners.p, "refine corners");
  }

  // resize image to detect more corners
  find_corners_reiszed(img, corners, params);
  if(params.show_processing) {
    printf("Merging corners (%d x %d) ... %lu\n", img.cols, img.rows, corners.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img, corners.p, "merge corners");
  }

  // polynomial fit
  if(params.polynomial_fit) {
    polynomial_fit(img_norm, corners, params);
    if(params.show_processing) {
      printf("Polyfitting corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
    }
    if(params.show_debug_image) {
      plot_corners(img, corners.p, "polynomial fit corners");
    }
  }

  // score corners
  sorce_corners(img_norm, img_weight, corners, params);

  // remove low scoring corners
  remove_low_scoring_corners(params.score_thr, corners, params);

  // non maximum suppression
  non_maximum_suppression_sparse(corners, 3, img.size(), params);
  if(params.show_processing) {
    printf("Scoring corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  }
  if(params.show_debug_image) {
    plot_corners(img, corners.p, "scoring corners");
  }
}

} // namespace cbdetect

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

#include "non_maximum_suppression.h"
#include <cstring>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

void non_maximum_suppression(const cv::Mat &img, int n, double tau, int margin, Corner &corners) {
  for (int i = n + margin; i < img.cols - n - margin; i += n + 1) {
    for (int j = n + margin; j < img.rows - n - margin; j += n + 1) {
      int maxi = i, maxj = j;
      double maxval = img.at<double>(j, i);

      for (int j2 = j; j2 <= j + n; ++j2) {
        for (int i2 = i; i2 <= i + n; ++i2) {
          if (img.at<double>(j2, i2) > maxval) {
            maxi = i2;
            maxj = j2;
            maxval = img.at<double>(j2, i2);
          }
        }
      }

      // maximum
      for (int j2 = maxj - n; j2 <= std::min(maxj + n, img.rows - 1 - margin); ++j2) {
        for (int i2 = maxi - n; i2 <= std::min(maxi + n, img.cols - 1 - margin); ++i2) {
          // origin code is -> img.at<double>(j2, i2) > maxval && (i2 < i || i2 > i + n || j2 < j || j2 > j + n)
          // I think the second criterion is redundant
          if (img.at<double>(j2, i2) > maxval) {
            goto GOTO_FAILED;
          }
        }
      }
      if (maxval > tau) {
        corners.p.emplace_back(cv::Point2d(maxi, maxj));
        corners.r.emplace_back(margin);
      }
      GOTO_FAILED:;
    }
  }
}

void non_maximum_suppression_sparse(Corner &corners, int n, cv::Size img_size) {
  cv::Mat img_score = cv::Mat::zeros(img_size, CV_64F);
  cv::Mat used = cv::Mat::ones(img_size, CV_32S) * -1;
  for (int i = 0; i < corners.p.size(); ++i) {
    int u = std::round(corners.p[i].x);
    int v = std::round(corners.p[i].y);
    if (img_score.at<double>(v, u) < corners.score[i]) {
      img_score.at<double>(v, u) = corners.score[i];
      used.at<int>(v, u) = i;
    }
  }
  std::vector<cv::Point2d> corners_out_p, corners_out_v1, corners_out_v2;
  std::vector<double> corners_out_score;
  std::vector<int> corners_out_r;
  for (int i = 0; i < corners.p.size(); ++i) {
    int u = std::round(corners.p[i].x);
    int v = std::round(corners.p[i].y);
    double score = corners.score[i];
    if (used.at<int>(v, u) != i) { continue; }
    for (int j2 = v - n; j2 <= v + n; ++j2) {
      for (int i2 = u - n; i2 <= u + n; ++i2) {
        if (img_score.at<double>(j2, i2) > score && (i2 != u || j2 != v)) {
          goto GOTO_FAILED;
        }
      }
    }
    corners_out_p.emplace_back(corners.p[i]);
    corners_out_r.emplace_back(corners.r[i]);
    corners_out_v1.emplace_back(corners.v1[i]);
    corners_out_v2.emplace_back(corners.v2[i]);
    corners_out_score.emplace_back(corners.score[i]);
    GOTO_FAILED:;
  }
  corners.p = std::move(corners_out_p);
  corners.r = std::move(corners_out_r);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
  corners.score = std::move(corners_out_score);
}

}

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

/**
* Copyright (C) 2017-present, Facebook, Inc.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "get_init_location.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "create_correlation_patch.h"
#include "non_maximum_suppression.h"

namespace cbdetect {

// form https://github.com/facebookincubator/deltille
void hessian_response(const cv::Mat &img_in, cv::Mat &img_out) {
  const int rows = img_in.rows;
  const int cols = img_in.cols;
  const int stride = cols;

  // allocate output
  img_out = cv::Mat::zeros(rows, cols, CV_64F);

  // setup input and output pointer to be centered at 1,0 and 1,1 resp.
  auto *in = img_in.ptr<double>(1);
  auto *out = img_out.ptr<double>(1) + 1;

  /* move 3x3 window and convolve */
  for (int r = 1; r < rows - 1; ++r) {
    double v11, v12, v21, v22, v31, v32;
    /* fill in shift registers at the beginning of the row */
    v11 = in[-stride];
    v12 = in[1 - stride];
    v21 = in[0];
    v22 = in[1];
    v31 = in[+stride];
    v32 = in[1 + stride];
    /* move input pointer to (1,2) of the 3x3 square */
    in += 2;
    for (int c = 1; c < cols - 1; ++c) {
      /* fetch remaining values (last column) */
      const double v13 = in[-stride];
      const double v23 = *in;
      const double v33 = in[+stride];

      // compute 3x3 Hessian values from symmetric differences.
      double Lxx = (v21 - 2 * v22 + v23);
      double Lyy = (v12 - 2 * v22 + v32);
      double Lxy = (v13 - v11 + v31 - v33) / 4.0f;

      /* normalize and write out */
      *out = Lxx * Lyy - Lxy * Lxy;

      /* move window */
      v11 = v12;
      v12 = v13;
      v21 = v22;
      v22 = v23;
      v31 = v32;
      v32 = v33;

      /* move input/output pointers */
      in++;
      out++;
    }
    out += 2;
  }
}

void get_init_location(const cv::Mat &img, Corner &corners, const Params &params) {
  corners.p.clear();
  corners.r.clear();
  corners.v1.clear();
  corners.v2.clear();
  corners.score.clear();
  switch (params.detct_mode) {
    case TemplateMatchFast:
    case TemplateMatchSlow: {
      // templates and scales
      std::vector<double> tprops;
      if (params.detct_mode == TemplateMatchFast) {
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

      // for all scales do
      for (const auto &r : params.radius) {
        // filter image
        cv::Mat img_corners = cv::Mat::zeros(img.size(), CV_64F);
        cv::Mat img_corners_a1, img_corners_a2, img_corners_b1, img_corners_b2, img_corners_mu,
            img_corners_a, img_corners_b, img_corners_s1, img_corners_s2;

        for (int i = 0; i < tprops.size(); i += 2) {
          std::vector<cv::Mat> template_kernel(4); // a1, a2, b1, b2
          create_correlation_patch(template_kernel, tprops[i], tprops[i + 1], r);

          // filter image with current template
          cv::filter2D(img, img_corners_a1, -1, template_kernel[0], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
          cv::filter2D(img, img_corners_a2, -1, template_kernel[1], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
          cv::filter2D(img, img_corners_b1, -1, template_kernel[2], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
          cv::filter2D(img, img_corners_b2, -1, template_kernel[3], cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

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
        non_maximum_suppression(img_corners, 1, params.score_thr, r, corners);
      }
      break;
    }
    case HessianResponse: {
      cv::Mat gauss_img;
      cv::GaussianBlur(img, gauss_img, cv::Size(7, 7), 1.5, 1.5);
      cv::Mat hessian_img;
      hessian_response(gauss_img, hessian_img);
      double mn = 0, mx = 0;
      cv::minMaxIdx(hessian_img, &mn, &mx, NULL, NULL);
      hessian_img = cv::abs(hessian_img);
      double thr = std::abs(mn * params.hessian_thr);
      for (const auto &r : params.radius) {
        non_maximum_suppression(hessian_img, r, thr, r, corners);
      }
      break;
    }
    default:break;
  }
}

}

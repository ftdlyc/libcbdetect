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

#include "refine_corners.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "find_corners.h"
#include "find_modes_meanshift.h"
#include "weight_mask.h"

namespace cbdetect {

std::vector<std::vector<double>> edge_orientations(cv::Mat &img_angle, cv::Mat &img_weight) {
  // number of bins (histogram parameter)
  int n = 32;

  // convert angles from normals to directions
  img_angle.forEach<double>([](double &val, const int *pos) -> void {
    val += M_PI / 2;
    val = val >= M_PI ? val - M_PI : val;
  });

  // create histogram
  std::vector<double> angle_hist(n, 0);
  for (int i = 0; i < img_angle.cols; ++i) {
    for (int j = 0; j < img_angle.rows; ++j) {
      int bin = static_cast<int>(std::floor(img_angle.at<double>(j, i) / (M_PI / n)));
      angle_hist[bin] += img_weight.at<double>(j, i);
    }
  }

  // find modes of smoothed histogram
  auto modes = find_modes_meanshift(angle_hist, 1.5);

  // if only one or no mode => return invalid corner
  if (modes.size() <= 1) { return std::vector<std::vector<double>>(); }

  // compute orientation at modes
  // extract 2 strongest modes and sort by angle
  double angle_1 = modes[0].first * M_PI / n + M_PI / n / 2;
  double angle_2 = modes[1].first * M_PI / n + M_PI / n / 2;
  if (angle_1 > angle_2) { std::swap(angle_1, angle_2); }

  // compute angle between modes
  double delta_angle = std::min(angle_2 - angle_1, angle_1 + M_PI - angle_2);

  // if angle too small => return invalid corner
  if (delta_angle <= 0.3) { return std::vector<std::vector<double>>(); }

  // set statistics: orientations
  std::vector<std::vector<double>> v(2, std::vector<double>(2));
  v[0][0] = std::cos(angle_1);
  v[0][1] = std::sin(angle_1);
  v[1][0] = std::cos(angle_2);
  v[1][1] = std::sin(angle_2);
  return v;
}

void get_rect_subpixel(const cv::Mat &img, double u, double v, int r, cv::Mat &img_sub) {
  int iu = u;
  int iv = v;
  double du = u - iu;
  double dv = v - iv;
  double a00 = 1 - du - dv - du * dv;
  double a01 = du - du * dv;
  double a10 = dv - du * dv;
  double a11 = du * dv;

  img_sub.create(2 * r + 1, 2 * r + 1, CV_64F);
  for (int j = -r; j <= r; ++j) {
    for (int i = -r; i <= r; ++i) {
      img_sub.at<double>(j + r, i + r) =
          a00 * img.at<double>(iv + j, iu + i) + a01 * img.at<double>(iv + j, iu + i + 1) +
              a10 * img.at<double>(iv + j + 1, iu + i) + a11 * img.at<double>(iv + j + 1, iu + i + 1);
    }
  }
}

void refine_corners(const cv::Mat &img_du, const cv::Mat &img_dv, const cv::Mat &img_angle, const cv::Mat &img_weight,
                    const std::vector<int> radius, Corner &corners) {
  // maximum iterations and precision
  int max_iteration = 5;
  double eps = 0.01;

  int width = img_du.cols, height = img_du.rows;
  std::vector<cv::Point2d> corners_out_p, corners_out_v1, corners_out_v2;
  std::vector<int> corners_out_r;
  auto mask = weight_mask(radius);

  // for all corners do
  for (int i = 0; i < corners.p.size(); ++i) {
    // extract current corner location
    double u_init = corners.p[i].x;
    double v_init = corners.p[i].y;
    int r = corners.r[i];

    // estimate edge orientations (continue, if too close to border)
    if (u_init - r < 0 || u_init + r >= width - 1 || v_init - r < 0 || v_init + r >= height - 1) { continue; }
    cv::Mat img_angle_sub, img_weight_sub;
    get_rect_subpixel(img_angle, u_init, v_init, r, img_angle_sub);
    get_rect_subpixel(img_weight, u_init, v_init, r, img_weight_sub);
    img_weight_sub = img_weight_sub.mul(mask[r]);
    auto v = edge_orientations(img_angle_sub, img_weight_sub);

    // continue, if invalid edge orientations
    if (v.empty()) { continue; }

    //corner orientation refinement
    cv::Mat A1 = cv::Mat::zeros(2, 2, CV_64F);
    cv::Mat A2 = cv::Mat::zeros(2, 2, CV_64F);
    for (int j2 = v_init - r; j2 <= v_init + r; ++j2) {
      for (int i2 = u_init - r; i2 <= u_init + r; ++i2) {
        //pixel orientation vector
        double o_du = img_du.at<double>(j2, i2);
        double o_dv = img_dv.at<double>(j2, i2);
        double o_norm = std::sqrt(o_du * o_du + o_dv * o_dv);
        if (o_norm < 0.1) { continue; }
        double o_du_norm = o_du / o_norm;
        double o_dv_norm = o_dv / o_norm;

        // robust refinement of orientation 1
        if (std::abs(o_du_norm * v[0][0] + o_dv_norm * v[0][1]) < 0.25) {
          A1.at<double>(0, 0) += o_du * o_du;
          A1.at<double>(0, 1) += o_du * o_dv;
          A1.at<double>(1, 0) += o_du * o_dv;
          A1.at<double>(1, 1) += o_dv * o_dv;
        }

        // robust refinement of orientation 2
        if (std::abs(o_du_norm * v[1][0] + o_dv_norm * v[1][1]) < 0.25) {
          A2.at<double>(0, 0) += o_du * o_du;
          A2.at<double>(0, 1) += o_du * o_dv;
          A2.at<double>(1, 0) += o_du * o_dv;
          A2.at<double>(1, 1) += o_dv * o_dv;
        }
      }
    }

    // set new corner orientation
    cv::Mat eig_tmp1, eig_tmp2;
    cv::eigen(A1, eig_tmp1, eig_tmp2);
    v[0][0] = eig_tmp2.at<double>(1, 0);
    v[0][1] = eig_tmp2.at<double>(1, 1);
    cv::eigen(A2, eig_tmp1, eig_tmp2);
    v[1][0] = eig_tmp2.at<double>(1, 0);
    v[1][1] = eig_tmp2.at<double>(1, 1);

    if (v[0][0] * v[1][1] - v[0][1] * v[1][0] < 0) {
      std::swap(v[0][0], v[1][0]);
      std::swap(v[0][1], v[1][1]);
    }

    // corner location refinement
    double u_cur = u_init, v_cur = v_init, u_last = u_cur, v_last = v_cur;
    for (int num_it = 0; num_it < max_iteration; ++num_it) {
      cv::Mat G = cv::Mat::zeros(2, 2, CV_64F);
      cv::Mat b = cv::Mat::zeros(2, 1, CV_64F);

      // get subpixel gradiant
      cv::Mat img_du_sub, img_dv_sub;
      if (u_cur - r < 0 || u_cur + r >= width - 1 || v_cur - r < 0 || v_cur + r >= height - 1) { break; }
      get_rect_subpixel(img_du, u_cur, v_cur, r, img_du_sub);
      get_rect_subpixel(img_dv, u_cur, v_cur, r, img_dv_sub);

      for (int j2 = 0; j2 < 2 * r + 1; ++j2) {
        for (int i2 = 0; i2 < 2 * r + 1; ++i2) {
          // pixel orientation vector
          double o_du = img_du_sub.at<double>(j2, i2);
          double o_dv = img_dv_sub.at<double>(j2, i2);
          double o_norm = std::sqrt(o_du * o_du + o_dv * o_dv);
          if (o_norm < 0.1) { continue; }
          double o_du_norm = o_du / o_norm;
          double o_dv_norm = o_dv / o_norm;

          // do not consider center pixel
          if (i2 == r && j2 == r) { continue; }

          // robust subpixel corner estimation
          // compute rel. position of pixel and distance to vectors
          double w_u = i2 - r - ((i2 - r) * v[0][0] + (j2 - r) * v[0][1]) * v[0][0];
          double v_u = j2 - r - ((i2 - r) * v[0][0] + (j2 - r) * v[0][1]) * v[0][1];
          double d1 = std::sqrt(w_u * w_u + v_u * v_u);
          w_u = i2 - r - ((i2 - r) * v[1][0] + (j2 - r) * v[1][1]) * v[1][0];
          v_u = j2 - r - ((i2 - r) * v[1][0] + (j2 - r) * v[1][1]) * v[1][1];
          double d2 = std::sqrt(w_u * w_u + v_u * v_u);

          // if pixel corresponds with either of the vectors / directions
          if ((d1 < 3 && std::abs(o_du_norm * v[0][0] + o_dv_norm * v[0][1]) < 0.25) ||
              (d2 < 3 && std::abs(o_du_norm * v[1][0] + o_dv_norm * v[1][1]) < 0.25)) {
            G.at<double>(0, 0) += o_du * o_du;
            G.at<double>(0, 1) += o_du * o_dv;
            G.at<double>(1, 0) += o_du * o_dv;
            G.at<double>(1, 1) += o_dv * o_dv;
            b.at<double>(0, 0) += o_du * o_du * (i2 - r + u_cur) + o_du * o_dv * (j2 - r + v_cur);
            b.at<double>(1, 0) += o_du * o_dv * (i2 - r + u_cur) + o_dv * o_dv * (j2 - r + v_cur);
          }
        }
      }

      // set new corner location if G has full rank
      cv::Mat new_pos = G.inv() * b;
      u_last = u_cur;
      v_last = v_cur;
      u_cur = new_pos.at<double>(0, 0);
      v_cur = new_pos.at<double>(1, 0);
      double dist = std::sqrt((u_cur - u_last) * (u_cur - u_last) + (v_cur - v_last) * (v_cur - v_last));
      if (dist >= 3) {
        u_cur = u_last;
        v_cur = v_last;
        break;
      }
      if (dist <= eps) { break; }
    }

    // add to corners
    if (std::sqrt((u_cur - u_init) * (u_cur - u_init) + (v_cur - v_init) * (v_cur - v_init)) < std::max(r / 2, 3)) {
      corners_out_p.emplace_back(cv::Point2d(u_cur, v_cur));
      corners_out_r.emplace_back(r);
      corners_out_v1.emplace_back(cv::Point2d(v[0][0], v[0][1]));
      corners_out_v2.emplace_back(cv::Point2d(v[1][0], v[1][1]));
    }
  }
  corners.p = std::move(corners_out_p);
  corners.r = std::move(corners_out_r);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
}

}

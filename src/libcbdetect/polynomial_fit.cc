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

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"
#include "libcbdetect/get_image_patch.h"
#include "libcbdetect/polynomial_fit.h"

namespace cbdetect {

void create_gaussin_filter_kernel(cv::Mat& kernel, double sigma) {
  int r = std::round(3 * sigma);
  kernel.create(2 * r + 1, 2 * r + 1, CV_64F);
  double sum = 0.0;
  for(int i = -r; i <= r; ++i) {
    for(int j = -r; j <= r; ++j) {
      kernel.at<double>(i + r, j + r) = std::exp(-(i * i + j * j) / 2 / sigma / sigma);
      sum += kernel.at<double>(i + r, j + r);
    }
  }
  kernel /= sum;
}

// paper: ROCHADE Robust Checkerboard Advanced Detection for Camera Calibration, ECCV 2014
int create_cone_filter_kernel(cv::Mat& kernel, int r) {
  kernel.create(2 * r + 1, 2 * r + 1, CV_64F);
  double sum = 0.0;
  int nzs    = 0;
  for(int i = -r; i <= r; ++i) {
    for(int j = -r; j <= r; ++j) {
      kernel.at<double>(i + r, j + r) = std::max(0.0, r + 1 - std::sqrt(i * i + j * j));
      sum += kernel.at<double>(i + r, j + r);
      if(kernel.at<double>(i + r, j + r) < 1e-6) {
        ++nzs;
      }
    }
  }
  kernel /= sum;
  return nzs;
}

void polynomial_fit_saddle(const cv::Mat& img, int r, Corner& corners) {
  // maximum iterations and precision
  int max_iteration = 5;
  double eps        = 0.01;
  int width         = img.cols;
  int height        = img.rows;

  std::vector<cv::Point2d> corners_out_p, corners_out_v1, corners_out_v2;
  std::vector<int> corners_out_r;
  std::vector<int> choose(corners.p.size(), 0);

  // cone filter
  cv::Mat blur_kernel, blur_img, mask;
  create_cone_filter_kernel(blur_kernel, r);
  int nzs = create_cone_filter_kernel(mask, r);
  cv::filter2D(img, blur_img, -1, blur_kernel, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::Mat A((2 * r + 1) * (2 * r + 1) - nzs, 6, CV_64F);
  int A_row = 0;
  for(int j = -r; j <= r; ++j) {
    for(int i = -r; i <= r; ++i) {
      if(mask.at<double>(j + r, i + r) >= 1e-6) {
        A.at<double>(A_row, 0) = i * i;
        A.at<double>(A_row, 1) = j * j;
        A.at<double>(A_row, 2) = i * j;
        A.at<double>(A_row, 3) = i;
        A.at<double>(A_row, 4) = j;
        A.at<double>(A_row, 5) = 1;
        ++A_row;
      }
    }
  }
  cv::Mat invAtAAt = (A.t() * A).inv(cv::DECOMP_SVD) * A.t();

  // for all corners do
  cv::parallel_for_(cv::Range(0, corners.p.size()), [&](const cv::Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      double u_init = corners.p[i].x;
      double v_init = corners.p[i].y;
      double u_cur = u_init, v_cur = v_init;
      bool is_saddle_point = true;

      // fit f(x, y) = k0 * x^2 + k1 * y^2 + k2 * x * y + k3 * x + k4 * y + k5
      // coef: [k0; k1; k2; k3; k4; k5]
      for(int num_it = 0; num_it < max_iteration; ++num_it) {
        cv::Mat k, b;
        if(u_cur - r < 0 || u_cur + r >= width - 1 || v_cur - r < 0 || v_cur + r >= height - 1) {
          is_saddle_point = false;
          break;
        }
        get_image_patch_with_mask(blur_img, mask, u_cur, v_cur, r, b);
        k = invAtAAt * b;

        // check if it is still a saddle point
        double det = 4 * k.at<double>(0, 0) * k.at<double>(1, 0) - k.at<double>(2, 0) * k.at<double>(2, 0);
        if(det > 0) {
          is_saddle_point = false;
          break;
        }

        // saddle point is the corner
        double dx = (k.at<double>(2, 0) * k.at<double>(4, 0) - 2 * k.at<double>(1, 0) * k.at<double>(3, 0)) / det;
        double dy = (k.at<double>(2, 0) * k.at<double>(3, 0) - 2 * k.at<double>(0, 0) * k.at<double>(4, 0)) / det;

        u_cur += dx;
        v_cur += dy;

        double dist = std::sqrt((u_cur - u_init) * (u_cur - u_init) + (v_cur - v_init) * (v_cur - v_init));
        if(dist > r) {
          is_saddle_point = false;
          break;
        }
        if(std::sqrt(dx * dx + dy * dy) <= eps) {
          break;
        }
      }

      // add to corners
      if(is_saddle_point) {
        choose[i]    = 1;
        corners.p[i] = cv::Point2d(u_cur, v_cur);
      }
    }
  });

  for(int i = 0; i < corners.p.size(); ++i) {
    if(choose[i] == 1) {
      corners_out_p.emplace_back(corners.p[i]);
      corners_out_r.emplace_back(corners.r[i]);
      corners_out_v1.emplace_back(corners.v1[i]);
      corners_out_v2.emplace_back(corners.v2[i]);
    }
  }
  corners.p  = std::move(corners_out_p);
  corners.r  = std::move(corners_out_r);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
}

// paper: Deltille Grids for Geometric Camera Calibration, ICCV 2017
void polynomial_fit_monkey_saddle(const cv::Mat& img, int r, Corner& corners) {
  // maximum iterations and precision
  int max_iteration = 5;
  double eps        = 0.001;
  int width         = img.cols;
  int height        = img.rows;

  std::vector<cv::Point2d> corners_out_p, corners_out_v1, corners_out_v2, corners_out_v3;
  std::vector<int> corners_out_r;
  std::vector<int> choose(corners.p.size(), 0);

  // cone filter
  cv::Mat blur_kernel, blur_img, mask;
  create_cone_filter_kernel(blur_kernel, r);
  int nzs = create_cone_filter_kernel(mask, r);
  cv::filter2D(img, blur_img, -1, blur_kernel, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::Mat A((2 * r + 1) * (2 * r + 1) - nzs, 10, CV_64F);
  int A_row = 0;
  for(int j = -r; j <= r; ++j) {
    for(int i = -r; i <= r; ++i) {
      if(mask.at<double>(j + r, i + r) >= 1e-6) {
        A.at<double>(A_row, 0) = i * i * i;
        A.at<double>(A_row, 1) = i * i * j;
        A.at<double>(A_row, 2) = i * j * j;
        A.at<double>(A_row, 3) = j * j * j;
        A.at<double>(A_row, 4) = i * i;
        A.at<double>(A_row, 5) = i * j;
        A.at<double>(A_row, 6) = j * j;
        A.at<double>(A_row, 7) = i;
        A.at<double>(A_row, 8) = j;
        A.at<double>(A_row, 9) = 1;
        ++A_row;
      }
    }
  }
  cv::Mat invAtAAt = (A.t() * A).inv(cv::DECOMP_SVD) * A.t();

  // for all corners do
  cv::parallel_for_(cv::Range(0, corners.p.size()), [&](const cv::Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      double u_init = corners.p[i].x;
      double v_init = corners.p[i].y;
      double u_cur = u_init, v_cur = v_init;
      bool is_monkey_saddle = true;

      // fit f(x, y) = k0 * x^3 + k1 * x^2*y + k2* x*y^2 + k3 * y^3 + k4 * x^2 + k5 * x*y + k6 * y^2 + k7 * x + k8 * y + k9
      // coef: [k0; k1; k2; k3; k4; k5; k6; k7; k8; k9]
      for(int num_it = 0; num_it < max_iteration; ++num_it) {
        cv::Mat k, b;
        if(u_cur - r < 0 || u_cur + r >= width - 1 || v_cur - r < 0 || v_cur + r >= height - 1) {
          is_monkey_saddle = false;
          break;
        }
        get_image_patch_with_mask(blur_img, mask, u_cur, v_cur, r, b);
        k = invAtAAt * b;

        // check if it is still a monkey saddle point
        double det = 3 * (k.at<double>(0, 0) * k.at<double>(2, 0) + k.at<double>(1, 0) * k.at<double>(3, 0)) -
                     (k.at<double>(1, 0) * k.at<double>(1, 0) + k.at<double>(2, 0) * k.at<double>(2, 0));
        if(det > 0) {
          is_monkey_saddle = false;
          break;
        }

        // the monkey saddle point is a degenerate critical point where all of its second derivatives are zero
        cv::Mat tmp_a = (cv::Mat_<double>(3, 2) << 3.0 * k.at<double>(0, 0), k.at<double>(1, 0),
                         2.0 * k.at<double>(1, 0), 2.0 * k.at<double>(2, 0),
                         k.at<double>(2, 0), 3.0 * k.at<double>(3, 0));
        cv::Mat tmp_b = (cv::Mat_<double>(3, 1) << -k.at<double>(4, 0), -k.at<double>(5, 0), -k.at<double>(6, 0));
        cv::Mat tmp_x = (tmp_a.t() * tmp_a).inv() * tmp_a.t() * tmp_b;
        double dx     = tmp_x.at<double>(0, 0);
        double dy     = tmp_x.at<double>(1, 0);

        u_cur += dx;
        v_cur += dy;

        double dist = std::sqrt((u_cur - u_init) * (u_cur - u_init) + (v_cur - v_init) * (v_cur - v_init));
        if(dist > r) {
          is_monkey_saddle = false;
          break;
        }
        if(std::sqrt(dx * dx + dy * dy) <= eps) {
          break;
        }
      }

      // add to corners
      if(is_monkey_saddle) {
        choose[i]    = 1;
        corners.p[i] = cv::Point2d(u_cur, v_cur);
      }
    }
  });

  for(int i = 0; i < corners.p.size(); ++i) {
    if(choose[i] == 1) {
      corners_out_p.emplace_back(corners.p[i]);
      corners_out_r.emplace_back(corners.r[i]);
      corners_out_v1.emplace_back(corners.v1[i]);
      corners_out_v2.emplace_back(corners.v2[i]);
      corners_out_v3.emplace_back(corners.v3[i]);
    }
  }
  corners.p  = std::move(corners_out_p);
  corners.r  = std::move(corners_out_r);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
  corners.v3 = std::move(corners_out_v3);
}

void polynomial_fit(const cv::Mat& img, Corner& corners, const Params& params) {
  if(params.corner_type == SaddlePoint) {
    polynomial_fit_saddle(img, params.polynomial_fit_half_kernel_size, corners);
  } else if(params.corner_type == MonkeySaddlePoint) {
    polynomial_fit_monkey_saddle(img, params.polynomial_fit_half_kernel_size, corners);
  }
}

} // namespace cbdetect

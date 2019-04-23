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

#include <math.h>

#include "libcbdetect/create_correlation_patch.h"
#include "libcbdetect/config.h"

namespace cbdetect {

void create_correlation_patch(std::vector<cv::Mat>& template_kernel, double angle_1, double angle_2, int radius) {
  // width and height
  int width  = radius * 2 + 1;
  int height = radius * 2 + 1;

  // initialize template
  template_kernel[0] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[1] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[2] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[3] = cv::Mat::zeros(height, width, CV_64F);

  // midpoint
  int mu = radius + 1;
  int mv = radius + 1;

  // compute normals from angles
  double n1[2]{-std::sin(angle_1), std::cos(angle_1)};
  double n2[2]{-std::sin(angle_2), std::cos(angle_2)};

  // for all points in template do
  for(int u = 0; u < width; ++u) {
    for(int v = 0; v < height; ++v) {
      // vector
      int vec[2]{u + 1 - mu, v + 1 - mv};
      double dist = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

      // check on which side of the normals we are
      double s1 = vec[0] * n1[0] + vec[1] * n1[1];
      double s2 = vec[0] * n2[0] + vec[1] * n2[1];

      if(dist <= radius) {
        if(s1 <= -0.1 && s2 <= -0.1) {
          template_kernel[0].at<double>(v, u) = 1;
        } else if(s1 >= 0.1 && s2 >= 0.1) {
          template_kernel[1].at<double>(v, u) = 1;
        } else if(s1 <= -0.1 && s2 >= 0.1) {
          template_kernel[2].at<double>(v, u) = 1;
        } else if(s1 >= 0.1 && s2 <= -0.1) {
          template_kernel[3].at<double>(v, u) = 1;
        }
      }
    }
  }

  // normalize
  double sum = cv::sum(template_kernel[0])[0];
  if(sum > 1e-5) {
    template_kernel[0] /= sum;
  }
  sum = cv::sum(template_kernel[1])[0];
  if(sum > 1e-5) {
    template_kernel[1] /= sum;
  }
  sum = cv::sum(template_kernel[2])[0];
  if(sum > 1e-5) {
    template_kernel[2] /= sum;
  }
  sum = cv::sum(template_kernel[3])[0];
  if(sum > 1e-5) {
    template_kernel[3] /= sum;
  }
}

void create_correlation_patch(std::vector<cv::Mat>& template_kernel,
                              double angle_1, double angle_2, double angle_3, int radius) {
  // width and height
  int width  = radius * 2 + 1;
  int height = radius * 2 + 1;

  // initialize template
  template_kernel[0] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[1] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[2] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[3] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[4] = cv::Mat::zeros(height, width, CV_64F);
  template_kernel[5] = cv::Mat::zeros(height, width, CV_64F);

  // midpoint
  int mu = radius + 1;
  int mv = radius + 1;

  // compute normals from angles
  double n1[2]{-std::sin(angle_1), std::cos(angle_1)};
  double n2[2]{-std::sin(angle_2), std::cos(angle_2)};
  double n3[3]{-std::sin(angle_3), std::cos(angle_3)};

  // for all points in template do
  for(int u = 0; u < width; ++u) {
    for(int v = 0; v < height; ++v) {
      // vector
      int vec[2]{u + 1 - mu, v + 1 - mv};
      double dist = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

      // check on which side of the normals we are
      double s1 = vec[0] * n1[0] + vec[1] * n1[1];
      double s2 = vec[0] * n2[0] + vec[1] * n2[1];
      double s3 = vec[0] * n3[0] + vec[1] * n3[1];

      if(dist <= radius) {
        if(s1 >= -0.1 && s2 <= -0.1) {
          template_kernel[0].at<double>(v, u) = 1;
        } else if(s1 >= 0.1 && s3 >= 0.1) {
          template_kernel[1].at<double>(v, u) = 1;
        } else if(s2 <= -0.1 && s3 >= 0.1) {
          template_kernel[2].at<double>(v, u) = 1;
        } else if(s1 <= 0.1 && s2 >= -0.1) {
          template_kernel[3].at<double>(v, u) = 1;
        } else if(s1 <= 0.1 && s3 <= -0.1) {
          template_kernel[4].at<double>(v, u) = 1;
        } else if(s2 >= 0.1 && s3 <= -0.1) {
          template_kernel[5].at<double>(v, u) = 1;
        }
      }
    }
  }

  // normalize
  double sum = cv::sum(template_kernel[0])[0];
  if(sum > 1e-5) {
    template_kernel[0] /= sum;
  }
  sum = cv::sum(template_kernel[1])[0];
  if(sum > 1e-5) {
    template_kernel[1] /= sum;
  }
  sum = cv::sum(template_kernel[2])[0];
  if(sum > 1e-5) {
    template_kernel[2] /= sum;
  }
  sum = cv::sum(template_kernel[3])[0];
  if(sum > 1e-5) {
    template_kernel[3] /= sum;
  }
  sum = cv::sum(template_kernel[4])[0];
  if(sum > 1e-5) {
    template_kernel[4] /= sum;
  }
  sum = cv::sum(template_kernel[5])[0];
  if(sum > 1e-5) {
    template_kernel[5] /= sum;
  }
}

} // namespace cbdetect

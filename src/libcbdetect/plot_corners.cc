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

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"
#include "libcbdetect/plot_corners.h"

namespace cbdetect {

void plot_corners(const cv::Mat& img, const std::vector<cv::Point2d>& corners, const char* str) {
  cv::Mat img_show;
  if(img.channels() != 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
#else
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
#endif
  } else {
    img_show = img.clone();
  }
  for(int i = 0; i < corners.size(); ++i) {
    cv::circle(img_show, corners[i], 2, cv::Scalar(0, 0, 255), -1);
  }
  cv::imshow(str, img_show);
  cv::waitKey();
}

void plot_corners(const cv::Mat& img, const Corner& corners) {
  cv::Mat img_show;
  if(img.channels() != 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
#else
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
#endif
  } else {
    img_show = img.clone();
  }
  for(int i = 0; i < corners.p.size(); ++i) {
    cv::line(img_show, corners.p[i], corners.p[i] + 20 * corners.v1[i], cv::Scalar(255, 0, 0), 2);
    cv::line(img_show, corners.p[i], corners.p[i] + 20 * corners.v2[i], cv::Scalar(0, 255, 0), 2);
    if(!corners.v3.empty()) {
      cv::line(img_show, corners.p[i], corners.p[i] + 20 * corners.v3[i], cv::Scalar(0, 0, 255), 2);
    }
    cv::circle(img_show, corners.p[i], 3, cv::Scalar(0, 0, 255), -1);
    cv::putText(img_show, std::to_string(i), cv::Point2i(corners.p[i].x - 12, corners.p[i].y - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
  }
  cv::imshow("corners_img", img_show);
  // cv::imwrite("corners_img.png", img_show);
}

} // namespace cbdetect

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

#include "plot_chessboards.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "find_corners.h"

namespace cbdetect {

void plot_chessboards(const cv::Mat &img, const Corner &corners,
                      const std::vector<std::vector<std::vector<int>>> &chessboards) {
  cv::Mat img_show;
  cv::cvtColor(img, img_show, CV_GRAY2BGR);

  for (int n = 0; n < chessboards.size(); ++n) {
    const auto &chessboard = chessboards[n];
    // plot lines in color
    for (int i = 0; i < chessboard.size(); ++i) {
      for (int j = 0; j < chessboard[0].size() - 1; ++j) {
        cv::line(img_show, corners.p[chessboard[i][j]], corners.p[chessboard[i][j + 1]],
                 cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      }
    }
    for (int j = 0; j < chessboard[0].size(); ++j) {
      for (int i = 0; i < chessboard.size() - 1; ++i) {
        cv::line(img_show, corners.p[chessboard[i][j]], corners.p[chessboard[i + 1][j]],
                 cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      }
    }

    // plot lines in white
    for (int i = 0; i < chessboard.size(); ++i) {
      for (int j = 0; j < chessboard[0].size() - 1; ++j) {
        cv::line(img_show, corners.p[chessboard[i][j]], corners.p[chessboard[i][j + 1]],
                 cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      }
    }
    for (int j = 0; j < chessboard[0].size(); ++j) {
      for (int i = 0; i < chessboard.size() - 1; ++i) {
        cv::line(img_show, corners.p[chessboard[i][j]], corners.p[chessboard[i + 1][j]],
                 cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      }
    }

    // plot coordinate system
    cv::line(img_show, corners.p[chessboard[0][0]], corners.p[chessboard[0][1]],
             cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    cv::line(img_show, corners.p[chessboard[0][0]], corners.p[chessboard[1][0]],
             cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    // plot numbers
    cv::Point2d mean(0, 0);
    for (int i = 0; i < chessboard.size(); ++i) {
      for (int j = 0; j < chessboard[0].size(); ++j) {
        mean += corners.p[chessboard[i][j]];
      }
    }
    mean /= (double) (chessboard.size() * chessboard[0].size());
    cv::putText(img_show, std::to_string(n), mean,
                cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(0, 0, 255), 2);
  }

  cv::imshow("chessboards_img", img_show);
  cv::waitKey();
}

}

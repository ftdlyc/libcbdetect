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

#include "plot_deltilles.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

void plot_deltilles(const cv::Mat &img, const Corner &corners,
                    const std::vector<Deltille> &deltilles) {
  cv::Mat img_show;
  if (img.channels() != 3) {
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
  } else {
    img_show = img.clone();
  }

  for (int n = 0; n < deltilles.size(); ++n) {
    const auto &deltille = deltilles[n];

    for (int i = 1; i < deltille.idx.size() - 1; ++i) {
      for (int j = 1; j < deltille.idx[i].size() - 1; ++j) {
        if (deltille.idx[i][j] < 0) { continue; }
        // plot lines in color
        if (deltille.idx[i][j + 1] >= 0) {
          cv::line(img_show, corners.p[deltille.idx[i][j]], corners.p[deltille.idx[i][j + 1]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        if (deltille.idx[i + 1][j + 1] >= 0) {
          cv::line(img_show, corners.p[deltille.idx[i][j]], corners.p[deltille.idx[i + 1][j + 1]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        if (deltille.idx[i + 1][j] >= 0) {
          cv::line(img_show, corners.p[deltille.idx[i][j]], corners.p[deltille.idx[i + 1][j]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }

        // plot lines in white
        if (deltille.idx[i][j + 1] >= 0) {
          cv::line(img_show, corners.p[deltille.idx[i][j]], corners.p[deltille.idx[i][j + 1]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        if (deltille.idx[i + 1][j + 1] >= 0) {
          cv::line(img_show, corners.p[deltille.idx[i][j]], corners.p[deltille.idx[i + 1][j + 1]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        if (deltille.idx[i + 1][j] >= 0) {
          cv::line(img_show, corners.p[deltille.idx[i][j]], corners.p[deltille.idx[i + 1][j]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
      }
    }

    // plot coordinate system
    for (int i = 1; i < deltille.idx.size() * deltille.idx[0].size(); ++i) {
      int row = i / deltille.idx[0].size();
      int col = i % deltille.idx[0].size();
      if (deltille.idx[row][col] < 0 || col == deltille.idx[0].size() - 1 ||
          deltille.idx[row][col + 1] < 0 || deltille.idx[row + 1][col] < 0) { continue; }
      cv::line(img_show, corners.p[deltille.idx[row][col]], corners.p[deltille.idx[row][col + 1]],
               cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
      cv::line(img_show, corners.p[deltille.idx[row][col]], corners.p[deltille.idx[row + 1][col]],
               cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      break;
    }

    // plot numbers
    cv::Point2d mean(0.0, 0.0);
    for (int i = 1; i < deltille.idx.size() - 1; ++i) {
      for (int j = 1; j < deltille.idx[i].size() - 1; ++j) {
        if (deltille.idx[i][j] < 0) { continue; }
        mean += corners.p[deltille.idx[i][j]];
      }
    }
    mean /= (double) (deltille.num);
    mean.x -= 10;
    mean.y += 10;
    cv::putText(img_show, std::to_string(n), mean,
                cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(196, 196, 0), 2);
  }

  cv::imshow("deltilles_img", img_show);
  cv::waitKey();
}

}

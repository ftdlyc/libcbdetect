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

#include <vector>

#include <opencv2/opencv.hpp>

#include "libcbdetect/plot_boards.h"
#include "libcbdetect/config.h"

namespace cbdetect {

void plot_boards(const cv::Mat& img, const Corner& corners,
                 const std::vector<Board>& boards, const Params& params) {
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

  for(int n = 0; n < boards.size(); ++n) {
    const auto& board = boards[n];

    for(int i = 1; i < board.idx.size() - 1; ++i) {
      for(int j = 1; j < board.idx[i].size() - 1; ++j) {
        if(board.idx[i][j] < 0) {
          continue;
        }
        // plot lines in color
        if(board.idx[i][j + 1] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i][j + 1]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        if(params.corner_type == MonkeySaddlePoint && board.idx[i + 1][j + 1] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i + 1][j + 1]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        if(board.idx[i + 1][j] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i + 1][j]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }

        // plot lines in white
        if(board.idx[i][j + 1] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i][j + 1]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        if(params.corner_type == MonkeySaddlePoint && board.idx[i + 1][j + 1] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i + 1][j + 1]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        if(board.idx[i + 1][j] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i + 1][j]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
      }
    }

    // plot coordinate system
    for(int i = 1; i < board.idx.size() * board.idx[0].size(); ++i) {
      int row = i / board.idx[0].size();
      int col = i % board.idx[0].size();
      if(board.idx[row][col] < 0 || col == board.idx[0].size() - 1 ||
         board.idx[row][col + 1] < 0 || board.idx[row + 1][col] < 0) {
        continue;
      }
      cv::line(img_show, corners.p[board.idx[row][col]], corners.p[board.idx[row][col + 1]],
               cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
      cv::line(img_show, corners.p[board.idx[row][col]], corners.p[board.idx[row + 1][col]],
               cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      break;
    }

    // plot numbers
    cv::Point2d mean(0.0, 0.0);
    for(int i = 1; i < board.idx.size() - 1; ++i) {
      for(int j = 1; j < board.idx[i].size() - 1; ++j) {
        if(board.idx[i][j] < 0) {
          continue;
        }
        mean += corners.p[board.idx[i][j]];
      }
    }
    mean /= (double)(board.num);
    mean.x -= 10;
    mean.y += 10;
    cv::putText(img_show, std::to_string(n), mean,
                cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(196, 196, 0), 2);
  }

  cv::imshow("boards_img", img_show);
  cv::waitKey();
}

} // namespace cbdetect

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

#include "deltilles_from_corners.h"
#include <algorithm>
#include <chrono>
#include <random>
#include <vector>
#include "config.h"
#include "deltille_energy.h"
#include "filter_deltille.h"
#include "grow_deltille.h"
#include "init_deltille.h"

namespace cbdetect {

void debug_grow_process(const cv::Mat &img, const Corner &corners, const Deltille &deltille,
                        const std::vector<cv::Point2i> &proposal, int direction, bool type) {
  cv::Mat img_show;
  if (img.channels() != 3) {
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
  } else {
    img_show = img.clone();
  }

  cv::Point2d mean(0.0, 0.0);
  for (int i = 0; i < deltille.idx.size(); ++i) {
    for (int j = 0; j < deltille.idx[i].size(); ++j) {
      if (deltille.idx[i][j] < 0) { continue; }
      cv::circle(img_show, corners.p[deltille.idx[i][j]], 4, cv::Scalar(255, 0, 0), -1);
      cv::putText(img_show, std::to_string(deltille.idx[i][j]),
                  cv::Point2i(corners.p[deltille.idx[i][j]].x - 12, corners.p[deltille.idx[i][j]].y - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
      mean += corners.p[deltille.idx[i][j]];
    }
  }
  mean /= (double) (deltille.num);
  mean.x -= 10;
  mean.y += 10;
  cv::putText(img_show, std::to_string(direction), mean,
              cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(196, 196, 0), 2);

  for (const auto &i : proposal) {
    if (deltille.idx[i.y][i.x] < 0) { continue; }
    if (type) {
      cv::circle(img_show, corners.p[deltille.idx[i.y][i.x]], 4, cv::Scalar(0, 255, 0), -1);
    } else {
      cv::circle(img_show, corners.p[deltille.idx[i.y][i.x]], 4, cv::Scalar(0, 0, 255), -1);
    }
  }

  cv::imshow("grow_process", img_show);
  cv::waitKey();
}

void deltilles_from_corners(const cv::Mat &img, const Corner &corners,
                            std::vector<Deltille> &deltilles, const Params &params) {
  // intialize deltilles
  deltilles.clear();
  Deltille deltille;
  std::vector<int> used(corners.p.size(), 0);

  // start from random index
  std::default_random_engine e;
  auto time = std::chrono::system_clock::now().time_since_epoch();
  e.seed(time.count());
  int start = e() % corners.p.size();

  // for all seed corners do
  int n = 0;
  while (n++ < corners.p.size()) {
    // init 3x3 deltille from seed i
    int i = (n + start) % corners.p.size();
    if (used[i] == 1 || !init_deltille(corners, i, deltille)) { continue; }

    // check if this is a useful initial guess
    cv::Point3i maxE_pos = deltille_energy(corners, deltille);
    double energy = deltille.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if (energy > -6.0) { continue; }

    for (int ii = 0; ii < deltille.idx.size(); ++ii) {
      for (int jj = 0; jj < deltille.idx[ii].size(); ++jj) {
        used[deltille.idx[ii][jj]] = 1;
      }
    }

    // grow chessboards
    while (1) {
      int num_corners = deltille.num;

      for (int j = 0; j < 6; ++j) {
        std::vector<cv::Point2i> proposal;
        GrowType grow_type = grow_deltille(corners, used, deltille, proposal, j);
        if (grow_type == Failure) { continue; }

        if (params.show_grow_processing) {
          for (int ii = 0; ii < deltille.idx.size(); ++ii) {
            for (int jj = 0; jj < deltille.idx[ii].size(); ++jj) {
              std::cout << deltille.idx[ii][jj] << " ";
            }
            std::cout << "\n";
          }
          std::cout << "\n";
          debug_grow_process(img, corners, deltille, proposal, j, false);
        }

        filter_deltille(corners, used, deltille, proposal, energy);

        if (params.show_grow_processing) {
          for (int ii = 0; ii < deltille.idx.size(); ++ii) {
            for (int jj = 0; jj < deltille.idx[ii].size(); ++jj) {
              std::cout << deltille.idx[ii][jj] << " ";
            }
            std::cout << "\n";
          }
          std::cout << "\n";
          debug_grow_process(img, corners, deltille, proposal, j, true);
        }

        if (grow_type == Inside) { --j; }
      }

      // exit loop
      if (deltille.num == num_corners) { break; }
    }

    deltilles.emplace_back(deltille);
  }
}

}
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

#include "init_chessboard.h"
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

int directional_neighbor(const Corner &corners, const std::vector<std::vector<int>> &chessboard,
                         const std::vector<int> &used, int idx, const cv::Point2d &v, double &min_dist) {
  std::vector<double> dists(corners.p.size(), 1e10);

  // distances
  for (int i = 0; i < corners.p.size(); ++i) {
    if (used[i]) { continue; }
    cv::Point2d dir = corners.p[i] - corners.p[idx];
    double dist_point = dir.x * v.x + dir.y * v.y;
    dir = dir - dist_point * v;
    double dist_edge = cv::norm(dir);
    double dist = dist_point + 5 * dist_edge;
    if (dist_point >= 0) { dists[i] = dist; }
  }

  // find best neighbor
  int neighbor_idx = std::min_element(dists.begin(), dists.end()) - dists.begin();
  min_dist = dists[neighbor_idx];
  return neighbor_idx;
}

void init_chessboard(const Corner &corners, int idx, std::vector<std::vector<int>> &chessboard) {
  chessboard.clear();
  // return if not enough corners
  if (corners.p.size() < 9) { return; }

  // init chessboard hypothesis
  chessboard = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  // extract feature index and orientation (central element)
  const cv::Point2d &v1 = corners.v1[idx];
  const cv::Point2d &v2 = corners.v3.empty() ? corners.v2[idx] : corners.v3[idx];
  chessboard[1][1] = idx;
  std::vector<int> used(corners.p.size(), 0);
  used[idx] = 1;
  double min_dist[8];

  // find left/right/top/bottom neighbors
  chessboard[1][0] = directional_neighbor(corners, chessboard, used, idx, -v1, min_dist[0]);
  used[chessboard[1][0]] = 1;
  chessboard[1][2] = directional_neighbor(corners, chessboard, used, idx, v1, min_dist[1]);
  used[chessboard[1][2]] = 1;
  chessboard[0][1] = directional_neighbor(corners, chessboard, used, idx, -v2, min_dist[2]);
  used[chessboard[0][1]] = 1;
  chessboard[2][1] = directional_neighbor(corners, chessboard, used, idx, v2, min_dist[3]);
  used[chessboard[2][1]] = 1;

  // find top-left/top-right/bottom-left/bottom-right neighbors
  int tmp1, tmp2;
  double d1, d2, min_dist_tmp1, min_dist_tmp2;
  tmp1 = directional_neighbor(corners, chessboard, used, chessboard[1][0], -v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, chessboard, used, chessboard[0][1], -v1, min_dist_tmp2);
  if (tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[chessboard[1][0]]) -
        cv::norm(corners.p[tmp1] - corners.p[chessboard[0][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[chessboard[1][0]]) -
        cv::norm(corners.p[tmp2] - corners.p[chessboard[0][1]]));
    if (d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  chessboard[0][0] = tmp1;
  min_dist[4] = min_dist_tmp1;
  used[tmp1] = 1;

  tmp1 = directional_neighbor(corners, chessboard, used, chessboard[1][2], -v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, chessboard, used, chessboard[0][1], v1, min_dist_tmp2);
  if (tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[chessboard[1][2]]) -
        cv::norm(corners.p[tmp1] - corners.p[chessboard[0][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[chessboard[1][2]]) -
        cv::norm(corners.p[tmp2] - corners.p[chessboard[0][1]]));
    if (d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  chessboard[0][2] = tmp1;
  min_dist[5] = min_dist_tmp1;
  used[tmp1] = 1;

  tmp1 = directional_neighbor(corners, chessboard, used, chessboard[1][0], v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, chessboard, used, chessboard[2][1], -v1, min_dist_tmp2);
  if (tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[chessboard[1][0]]) -
        cv::norm(corners.p[tmp1] - corners.p[chessboard[2][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[chessboard[1][0]]) -
        cv::norm(corners.p[tmp2] - corners.p[chessboard[2][1]]));
    if (d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  chessboard[2][0] = tmp1;
  min_dist[6] = min_dist_tmp1;
  used[tmp1] = 1;

  tmp1 = directional_neighbor(corners, chessboard, used, chessboard[1][2], v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, chessboard, used, chessboard[2][1], v1, min_dist_tmp2);
  if (tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[chessboard[1][2]]) -
        cv::norm(corners.p[tmp1] - corners.p[chessboard[2][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[chessboard[1][2]]) -
        cv::norm(corners.p[tmp2] - corners.p[chessboard[2][1]]));
    if (d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  chessboard[2][2] = tmp1;
  min_dist[7] = min_dist_tmp1;

  // initialization must be homogenously distributed
  for (int i = 0; i < 8; ++i) {
    if (std::abs(min_dist[i] - 1e10) < 1) {
      chessboard.clear();
    }
  }
}

}

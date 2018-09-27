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

#include "grow_chessboard.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "find_corners.h"

namespace cbdetect {

// linear prediction (old)
// function pred = predict_corners(p1,p2,p3)
// pred = 2*p3-p2;
//
// replica prediction (new)
std::vector<cv::Point2d> predict_corners(const Corner &corners,
                                         const std::vector<int> &p1,
                                         const std::vector<int> &p2,
                                         const std::vector<int> &p3) {
  std::vector<cv::Point2d> pred(p1.size());
  for (int i = 0; i < pred.size(); ++i) {
    // compute vectors
    cv::Point2d v1 = corners.p[p2[i]] - corners.p[p1[i]];
    cv::Point2d v2 = corners.p[p3[i]] - corners.p[p2[i]];

    // predict angles
    double a1 = std::atan2(v1.y, v1.x);
    double a2 = std::atan2(v2.y, v2.x);
    double a3 = 2 * a2 - a1;

    //  predict scales
    double s1 = cv::norm(v1);
    double s2 = cv::norm(v2);
    double s3 = 2 * s2 - s1;

    // predict p4 (the factor 0.75 ensures that under extreme
    // distortions (omnicam) the closer prediction is selected)
    pred[i].x = corners.p[p3[i]].x + 0.75 * s3 * std::cos(a3);
    pred[i].y = corners.p[p3[i]].y + 0.75 * s3 * std::sin(a3);
  }
  return pred;
}

std::vector<int> assign_closest_corners(const Corner &corners,
                                        std::vector<int> &used,
                                        const std::vector<cv::Point2d> &pred,
                                        const std::vector<int> &last) {
  std::vector<int> idx(pred.size());
  // return error if not enough candidates are available
  int n_unused = 0;
  for (const auto &i : used) {
    if (i == 0) { n_unused++; }
  }
  if (n_unused < pred.size()) { return idx; }

  // build distance matrix
  std::vector<std::vector<double>> D(pred.size(), std::vector<double>(corners.p.size(), 1e10));
  for (int i = 0; i < pred.size(); ++i) {
    cv::Point2d w = pred[i] - corners.p[last[i]];
    for (int j = 0; j < corners.p.size(); ++j) {
      if (used[j] == 1) { continue; }
      cv::Point2d v_tmp = corners.p[j] - corners.p[last[i]];
      cv::Point2d v(v_tmp.dot(w), v_tmp.dot(cv::Point2d(w.y, -w.x)));
      v = v / (cv::norm(w) * cv::norm(w));
      double d1 = std::atan2(v.y, v.x);
      double d2 = (1 - cv::norm(v));
      D[i][j] = std::sqrt(std::abs(d1) + d2 * d2 * d2 * d2);
    }
  }

  // search for closest corners
  for (int i = 0; i < pred.size(); ++i) {
    double min_D = 1e10;
    int min_row = 0;
    int min_col = 0;
    for (int j = 0; j < pred.size(); ++j) {
      int min_row_2 = std::min_element(D[j].begin(), D[j].end()) - D[j].begin();
      if (D[j][min_row_2] < min_D) {
        min_D = D[j][min_row_2];
        min_row = min_row_2;
        min_col = j;
      }
    }
    for (auto &j : D[min_col]) { j = 1e10; }
    for (int j = 0; j < pred.size(); ++j) {
      D[j][min_row] = 1e10;
    }
    idx[min_col] = min_row;
    used[min_row] = 1;
  }

  return idx;
}

void grow_chessboard(const Corner &corners, std::vector<std::vector<int>> &chessboard, int border_type) {
  // return immediately, if there do not exist any chessboards
  if (chessboard.empty()) { return; }

  // list of unused feature elements
  std::vector<int> used(corners.p.size(), 0);
  for (int i = 0; i < chessboard.size(); ++i) {
    for (int j = 0; j < chessboard[0].size(); ++j) {
      used[chessboard[i][j]] = 1;
    }
  }

  // switch border type 1..4
  std::vector<cv::Point2d> pred;
  std::vector<int> idx, p1, p2, p3;
  int rows = chessboard.size();
  int cols = chessboard[0].size();
  switch (border_type) {
    case 0: {
      p1.resize(rows);
      p2.resize(rows);
      p3.resize(rows);
      for (int i = 0; i < rows; ++i) {
        p1[i] = chessboard[i][cols - 3];
        p2[i] = chessboard[i][cols - 2];
        p3[i] = chessboard[i][cols - 1];
      }
      pred = std::move(predict_corners(corners, p1, p2, p3));
      idx = std::move(assign_closest_corners(corners, used, pred, p3));
      if (!idx.empty()) {
        for (int i = 0; i < rows; ++i) {
          chessboard[i].emplace_back(idx[i]);
        }
      }
      break;
    }
    case 1: {
      p1.resize(cols);
      p2.resize(cols);
      p3.resize(cols);
      for (int i = 0; i < cols; ++i) {
        p1[i] = chessboard[rows - 3][i];
        p2[i] = chessboard[rows - 2][i];
        p3[i] = chessboard[rows - 1][i];
      }
      pred = std::move(predict_corners(corners, p1, p2, p3));
      idx = std::move(assign_closest_corners(corners, used, pred, p3));
      if (!idx.empty()) {
        chessboard.emplace_back(idx);
      }
      break;
    }
    case 2: {
      p1.resize(rows);
      p2.resize(rows);
      p3.resize(rows);
      for (int i = 0; i < rows; ++i) {
        p1[i] = chessboard[i][2];
        p2[i] = chessboard[i][1];
        p3[i] = chessboard[i][0];
      }
      pred = std::move(predict_corners(corners, p1, p2, p3));
      idx = std::move(assign_closest_corners(corners, used, pred, p3));
      if (!idx.empty()) {
        for (int i = 0; i < rows; ++i) {
          chessboard[i].insert(chessboard[i].begin(), idx[i]);
        }
      }
      break;
    }
    case 3: {
      p1.resize(cols);
      p2.resize(cols);
      p3.resize(cols);
      for (int i = 0; i < cols; ++i) {
        p1[i] = chessboard[2][i];
        p2[i] = chessboard[1][i];
        p3[i] = chessboard[0][i];
      }
      pred = std::move(predict_corners(corners, p1, p2, p3));
      idx = std::move(assign_closest_corners(corners, used, pred, p3));
      if (!idx.empty()) {
        chessboard.insert(chessboard.begin(), idx);
      }
      break;
    }
    default:;
  }
}

}

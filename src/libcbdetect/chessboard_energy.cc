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

#include "chessboard_energy.h"
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

double chessboard_energy(const Corner &corners, const std::vector<std::vector<int>> &chessboard) {
  // energy: number of corners
  double E_corners = -1.0 * chessboard.size() * chessboard[0].size();

  // energy: structure
  double E_structure = 0.;

  // walk through rows
  for (int i = 0; i < chessboard.size(); ++i) {
    for (int j = 0; j < chessboard[0].size() - 2; ++j) {
      const cv::Point2d &x1 = corners.p[chessboard[i][j]];
      const cv::Point2d &x2 = corners.p[chessboard[i][j + 1]];
      const cv::Point2d &x3 = corners.p[chessboard[i][j + 2]];
      E_structure = std::max(E_structure, cv::norm(x1 + x3 - 2 * x2) / cv::norm(x1 - x3));
    }
  }

  // walk through columns
  for (int j = 0; j < chessboard[0].size(); ++j) {
    for (int i = 0; i < chessboard.size() - 2; ++i) {
      const cv::Point2d &x1 = corners.p[chessboard[i][j]];
      const cv::Point2d &x2 = corners.p[chessboard[i + 1][j]];
      const cv::Point2d &x3 = corners.p[chessboard[i + 2][j]];
      E_structure = std::max(E_structure, cv::norm(x1 + x3 - 2 * x2) / cv::norm(x1 - x3));

    }
  }

  // final energy
  return E_corners * (1 - E_structure);
}

}

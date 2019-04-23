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

#include "libcbdetect/board_energy.h"
#include "libcbdetect/config.h"
#include "libcbdetect/filter_board.h"

namespace cbdetect {

double find_minE(const Board& board, const cv::Point2i& p) {
  double minE = std::min(std::min(board.energy[p.y][p.x][0], board.energy[p.y][p.x][1]),
                         board.energy[p.y][p.x][2]);
  if(p.x - 1 >= 0) {
    minE = std::min(minE, board.energy[p.y][p.x - 1][0]);
  }
  if(p.x - 1 >= 0 && p.y - 1 >= 0) {
    minE = std::min(minE, board.energy[p.y - 1][p.x - 1][1]);
  }
  if(p.y - 1 >= 0) {
    minE = std::min(minE, board.energy[p.y - 1][p.x][2]);
  }
  if(p.x - 2 >= 0) {
    minE = std::min(minE, board.energy[p.y][p.x - 2][0]);
  }
  if(p.x - 2 >= 0 && p.y - 2 >= 0) {
    minE = std::min(minE, board.energy[p.y - 2][p.x - 2][1]);
  }
  if(p.y - 2 >= 0) {
    minE = std::min(minE, board.energy[p.y - 2][p.x][2]);
  }
  return minE;
}

void filter_board(const Corner& corners, std::vector<int>& used, Board& board,
                  std::vector<cv::Point2i>& proposal, double& energy, const Params& params) {
  // erase wrong corners
  while(!proposal.empty()) {
    cv::Point3i maxE_pos = board_energy(corners, board, params);
    double p_energy      = board.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if(p_energy <= energy) {
      energy = p_energy;
      break;
    }
    if(params.corner_type == SaddlePoint && !params.occlusion) {
      for(const auto& p : proposal) {
        used[board.idx[p.y][p.x]] = 0;
        board.idx[p.y][p.x]       = -2;
        --board.num;
      }
      return;
    }

    // find the wrongest corner
    cv::Point2i p[3];
    p[0] = {maxE_pos.x, maxE_pos.y};
    switch(maxE_pos.z) {
    case 0: {
      p[1] = {maxE_pos.x + 1, maxE_pos.y};
      p[2] = {maxE_pos.x + 2, maxE_pos.y};
      break;
    }
    case 1: {
      p[1] = {maxE_pos.x + 1, maxE_pos.y + 1};
      p[2] = {maxE_pos.x + 2, maxE_pos.y + 2};
      break;
    }
    case 2: {
      p[1] = {maxE_pos.x, maxE_pos.y + 1};
      p[2] = {maxE_pos.x, maxE_pos.y + 2};
      break;
    }
    default:
      break;
    }
    double minE_wrong[3];
    minE_wrong[0] = find_minE(board, p[0]);
    minE_wrong[1] = find_minE(board, p[1]);
    minE_wrong[2] = find_minE(board, p[2]);

    double minE = -DBL_MAX;
    int iter    = 0;
    for(auto it = proposal.begin(); it < proposal.end(); ++it) {
      if(it->x == p[0].x && it->y == p[0].y && minE_wrong[0] > minE) {
        minE       = minE_wrong[0];
        maxE_pos.x = it->x;
        maxE_pos.y = it->y;
        iter       = it - proposal.begin();
      }
      if(it->x == p[1].x && it->y == p[1].y && minE_wrong[1] > minE) {
        minE       = minE_wrong[1];
        maxE_pos.x = it->x;
        maxE_pos.y = it->y;
        iter       = it - proposal.begin();
      }
      if(it->x == p[2].x && it->y == p[2].y && minE_wrong[2] > minE) {
        minE       = minE_wrong[2];
        maxE_pos.x = it->x;
        maxE_pos.y = it->y;
        iter       = it - proposal.begin();
      }
    }

    proposal.erase(proposal.begin() + iter);
    used[board.idx[maxE_pos.y][maxE_pos.x]] = 0;
    board.idx[maxE_pos.y][maxE_pos.x]       = -2;
    --board.num;
  }
}

} // namespace cbdetect

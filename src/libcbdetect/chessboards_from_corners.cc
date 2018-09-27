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

#include "chessboard_energy.h"
#include <algorithm>
#include <vector>
#include "chessboards_from_corners.h"
#include "init_chessboard.h"
#include "find_corners.h"
#include "grow_chessboard.h"

namespace cbdetect {

void chessboards_from_corners(const Corner &corners, std::vector<std::vector<std::vector<int>>> &chessboards) {
  // intialize chessboards
  chessboards.clear();
  std::vector<std::vector<int>> chessboard;

  // for all seed corners do
  for (int i = 0; i < corners.p.size(); i += 3) {
    // init 3x3 chessboard from seed i
    init_chessboard(corners, i, chessboard);

    // check if this is a useful initial guess
    if (chessboard.empty() || chessboard_energy(corners, chessboard) > 0) { continue; }

    // grow chessboards
    while (1) {
      // compute proposals and energies
      std::vector<std::vector<std::vector<int>>> proposal(4, chessboard);
      double p_energy[4];
      for (int j = 0; j < 4; ++j) {
        grow_chessboard(corners, proposal[j], j);
        p_energy[j] = chessboard_energy(corners, proposal[j]);
      }

      // find best proposal
      int min_idx = std::min_element(std::begin(p_energy), std::end(p_energy)) - std::begin(p_energy);

      // accept best proposal, if energy is reduced
      if (p_energy[min_idx] < chessboard_energy(corners, chessboard)) {
        chessboard = std::move(proposal[min_idx]);
      } else {
        // otherwise exit loop
        break;
      }
    }

    double energy = chessboard_energy(corners, chessboard);
    // if chessboard has low energy (corresponding to high quality)
    if (energy < -10) {
      std::vector<std::pair<int, double>> overlap;
      for (int j = 0; j < chessboards.size(); ++j) {
        // check if new chessboard proposal overlaps with existing chessboards
        for (int k1 = 0; k1 < chessboard.size(); ++k1) {
          for (int k2 = 0; k2 < chessboard[0].size(); ++k2) {
            for (int l1 = 0; l1 < chessboards[j].size(); ++l1) {
              for (int l2 = 0; l2 < chessboards[j][0].size(); ++l2) {
                if (chessboard[k1][k2] == chessboards[j][l1][l2]) {
                  overlap.emplace_back(std::make_pair(j, chessboard_energy(corners, chessboards[j])));
                  goto GOTO_BREAK;
                }
              }
            }
          }
        }
        GOTO_BREAK:;
      }


      // add chessboard (and replace overlapping if neccessary)
      if (overlap.empty()) {
        chessboards.emplace_back(chessboard);
      } else {
        bool is_better = true;
        for (int j = 0; j < overlap.size(); ++j) {
          if (overlap[j].second <= energy) {
            is_better = false;
            break;
          }
        }
        if (is_better) {
          std::vector<std::vector<std::vector<int>>> tmp;
          for (int j = 0, k = 0; j < chessboards.size(); ++j) {
            if (overlap[k].first == j) {
              continue;
              ++k;
            }
            tmp.emplace_back(chessboards[j]);
          }
          std::swap(tmp, chessboards);
          chessboards.emplace_back(chessboard);
        }
      }
    }
  }
}

}

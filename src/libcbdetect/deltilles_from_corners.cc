// c++ version by ftdlyc

#include "deltilles_from_corners.h"
#include <algorithm>
#include <vector>
#include "config.h"
#include "deltille_energy.h"
#include "filter_deltille.h"
#include "grow_deltille.h"
#include "init_deltille.h"

namespace cbdetect {

void deltilles_from_corners(const Corner &corners, std::vector<Deltille> &deltilles) {
  // intialize deltilles
  deltilles.clear();
  Deltille deltille;
  std::vector<int> used(corners.p.size(), 0);

  // for all seed corners do
  for (int i = 0; i < corners.p.size(); i += 3) {
    // init 3x3 deltille from seed i
    if (used[i] == 1 || !init_deltille(corners, i, deltille)) { continue; }

    // check if this is a useful initial guess
    cv::Point3i maxE_pos = deltille_energy(corners, deltille);
    double energy = deltille.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if (energy > 0) { continue; }

    for (int ii = 0; ii < deltille.idx.size(); ++ii) {
      for (int jj = 0; jj < deltille.idx[ii].size(); ++jj) {
        used[deltille.idx[ii][jj]] = 1;
      }
    }

    // grow chessboards
    while (1) {
      int num_corners = deltille.num;
      for (int j = 0; j < 4; ++j) {
        std::vector<cv::Point2i> proposal;
        if (!grow_deltille(corners, used, deltille, proposal, j)) { continue; }

        for (int ii = 0; ii < deltille.idx.size(); ++ii) {
          for (int jj = 0; jj < deltille.idx[ii].size(); ++jj) {
            std::cout << deltille.idx[ii][jj] << " ";
          }
          std::cout << "\n";
        }
        std::cout << "\n";
        filter_deltille(corners, used, deltille, proposal, energy);

        for (int ii = 0; ii < deltille.idx.size(); ++ii) {
          for (int jj = 0; jj < deltille.idx[ii].size(); ++jj) {
            std::cout << deltille.idx[ii][jj] << " ";
          }
          std::cout << "\n";
        }
        std::cout << "\n";
      }
      // exit loop
      if (deltille.num == num_corners) { break; }
    }

  }

  int pp = 0;
}

}
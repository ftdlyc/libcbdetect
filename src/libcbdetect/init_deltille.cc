// c++ version by ftdlyc

#include "init_deltille.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "deltille_energy.h"
#include "init_chessboard.h"

namespace cbdetect {

bool init_deltille(const Corner &corners, int idx, Deltille &deltille) {
  init_chessboard(corners, idx, deltille.idx);
  if (deltille.idx.empty()) { return false; }
  deltille.num = 9;
  deltille.energy = std::move(
      std::vector<std::vector<std::vector<double>>>(3,
                                                    std::vector<std::vector<double>>(3,
                                                                                     std::vector<double>(3, DBL_MAX))));
  return true;
}

}

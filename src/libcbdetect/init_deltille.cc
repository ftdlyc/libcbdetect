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

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

#ifndef LIBCBDETECT_GROW_DELTILLE_H
#define LIBCBDETECT_GROW_DELTILLE_H

#include <vector>
#include "config.h"

namespace cbdetect {

enum GrowType {
  Failure = 0,
  Inside,
  Board,
};

LIBCBDETECT_DLL_DECL GrowType grow_deltille(const Corner &corners, std::vector<int> &used, Deltille &deltille,
                                            std::vector<cv::Point2i> &proposal, int board_type);

}

#endif //LIBCBDETECT_GROW_DELTILLE_H

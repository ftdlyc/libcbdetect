// c++ version by ftdlyc

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

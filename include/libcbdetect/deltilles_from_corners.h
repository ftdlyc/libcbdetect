// c++ version by ftdlyc

#ifndef LIBCBDETECT_DELTILLE_FROM_CORNERS_H
#define LIBCBDETECT_DELTILLE_FROM_CORNERS_H

#include <vector>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void deltilles_from_corners(const Corner &corners,
                                                 std::vector<std::vector<std::vector<int>>> &deltilles);

}

#endif //LIBCBDETECT_DELTILLE_FROM_CORNERS_H

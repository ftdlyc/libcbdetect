// c++ version by ftdlyc

#ifndef LIBCBDETECT_INIT_DELTILLE_H
#define LIBCBDETECT_INIT_DELTILLE_H

#include <vector>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void init_deltille(const Corner &corners, int idx, std::vector<std::vector<int>> &deltille);

}

#endif //LIBCBDETECT_INIT_DELTILLE_H

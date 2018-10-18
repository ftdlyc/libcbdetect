// c++ version by ftdlyc

#ifndef LIBCBDETECT_GROW_DELTILLE_H
#define LIBCBDETECT_GROW_DELTILLE_H

#include <vector>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void grow_deltille(const Corner &corners, std::vector<std::vector<int>> &deltille,
                                        int border_type);

}

#endif //LIBCBDETECT_GROW_DELTILLE_H

// c++ version by ftdlyc

#ifndef LIBCBDETECT_DELTILLE_ENERGY_H
#define LIBCBDETECT_DELTILLE_ENERGY_H

#include <vector>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL double deltille_energy(const Corner &corners, const std::vector<std::vector<int>> &deltille);

}

#endif //LIBCBDETECT_DELTILLE_ENERGY_H

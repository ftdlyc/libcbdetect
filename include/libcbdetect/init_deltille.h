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

#ifndef LIBCBDETECT_INIT_DELTILLE_H
#define LIBCBDETECT_INIT_DELTILLE_H

#include <vector>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL bool init_deltille(const Corner &corners, int idx, Deltille &deltille);

}

#endif //LIBCBDETECT_INIT_DELTILLE_H

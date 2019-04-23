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

/*
% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#include <math.h>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "libcbdetect/find_modes_meanshift.h"
#include "libcbdetect/config.h"

namespace cbdetect {

// efficient mean-shift approximation by histogram smoothing
std::vector<std::pair<int, double>> find_modes_meanshift(const std::vector<double>& hist, double sigma) {
  std::unordered_map<int, double> hash_table;
  std::vector<std::pair<int, double>> modes;

  int r = static_cast<int>(std::round(2 * sigma));
  std::vector<double> weight(2 * r + 1, 0);
  for(int i = 0; i < 2 * r + 1; ++i) {
    weight[i] = std::exp(-0.5 * (i - r) * (i - r) / sigma / sigma) / std::sqrt(2 * M_PI) / sigma;
  }

  // compute smoothed histogram
  int n = hist.size();
  std::vector<double> hist_smoothed(n, 0);
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < 2 * r + 1; ++j) {
      hist_smoothed[(i + r) % n] += hist[(i + j) % n] * weight[j];
    }
  }

  // check if at least one entry is non-zero
  // (otherwise mode finding may run infinitly)
  auto max_hist_val = std::max_element(hist_smoothed.begin(), hist_smoothed.end());
  if(*max_hist_val < 1e-6) {
    return modes;
  }

  // mode finding
  std::vector<int> visited(n, 0);
  for(int i = 0; i < n; ++i) {
    int j = i;
    if(!visited[j]) {
      while(1) {
        visited[j] = 1;
        int j1 = (j + 1) % n, j2 = (j + n - 1) % n;
        double h0 = hist_smoothed[j];
        double h1 = hist_smoothed[j1];
        double h2 = hist_smoothed[j2];
        if(h1 >= h0 && h1 >= h2) {
          j = j1;
        } else if(h2 > h0 && h2 > h1) {
          j = j2;
        } else {
          break;
        }
      }
      hash_table[j] = hist_smoothed[j];
    }
  }

  for(const auto& i : hash_table) {
    modes.emplace_back(i);
  }
  std::sort(modes.begin(), modes.end(), [](const auto& i1, const auto& i2) -> bool {
    return i1.second > i2.second;
  });

  return modes;
};

} // namespace cbdetect
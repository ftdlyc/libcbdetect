// c++ version by ftdlyc

#ifndef LIBCBDETECT_FILTER_DELTILLE_H
#define LIBCBDETECT_FILTER_DELTILLE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void filter_deltille(const Corner &corners, std::vector<int> &used, Deltille &deltille,
                                          std::vector<cv::Point2i> &proposal, double &energy);

}

#endif //LIBCBDETECT_FILTER_DELTILLE_H

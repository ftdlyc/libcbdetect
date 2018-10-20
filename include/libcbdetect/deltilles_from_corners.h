// c++ version by ftdlyc

#ifndef LIBCBDETECT_DELTILLE_FROM_CORNERS_H
#define LIBCBDETECT_DELTILLE_FROM_CORNERS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void deltilles_from_corners(const cv::Mat &img, const Corner &corners,
                                                 std::vector<Deltille> &deltilles, const Params &params);

}

#endif //LIBCBDETECT_DELTILLE_FROM_CORNERS_H

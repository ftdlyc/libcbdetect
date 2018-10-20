// c++ version by ftdlyc

#ifndef LIBCBDETECT_PLOT_DELTILLES_H
#define LIBCBDETECT_PLOT_DELTILLES_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void plot_deltilles(const cv::Mat &img, const Corner &corners,
                                         const std::vector<Deltille> &deltilles);

}

#endif //LIBCBDETECT_PLOT_DELTILLES_H

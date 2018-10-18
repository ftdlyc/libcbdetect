// c++ version by ftdlyc

#ifndef LIBCBDETECT_PLOT_DELTILLES_H
#define LIBCBDETECT_PLOT_DELTILLES_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void plot_chessboards(const cv::Mat &img, const Corner &corners,
                                           const std::vector<std::vector<std::vector<int>>> &deltilles);

}

#endif //LIBCBDETECT_PLOT_DELTILLES_H

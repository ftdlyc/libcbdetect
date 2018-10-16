// c++ version by ftdlyc

#ifndef LIBCBDETECT_GET_IMAGE_PATCH_H
#define LIBCBDETECT_GET_IMAGE_PATCH_H

#include <opencv2/opencv.hpp>

namespace cbdetect {

void get_image_patch(const cv::Mat &img, double u, double v, int r, cv::Mat &img_sub);

void get_image_patch_with_mask(const cv::Mat &img, const cv::Mat &mask, double u, double v, int r, cv::Mat &img_sub);

}

#endif //LIBCBDETECT_GET_IMAGE_PATCH_H

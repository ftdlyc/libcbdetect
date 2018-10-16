// c++ version by ftdlyc

#include "get_image_patch.h"
#include <opencv2/opencv.hpp>

namespace cbdetect {

void get_image_patch(const cv::Mat &img, double u, double v, int r, cv::Mat &img_sub) {
  int iu = u;
  int iv = v;
  double du = u - iu;
  double dv = v - iv;
  double a00 = 1 - du - dv + du * dv;
  double a01 = du - du * dv;
  double a10 = dv - du * dv;
  double a11 = du * dv;

  img_sub.create(2 * r + 1, 2 * r + 1, CV_64F);
  for (int j = -r; j <= r; ++j) {
    for (int i = -r; i <= r; ++i) {
      img_sub.at<double>(j + r, i + r) =
          a00 * img.at<double>(iv + j, iu + i) + a01 * img.at<double>(iv + j, iu + i + 1) +
              a10 * img.at<double>(iv + j + 1, iu + i) + a11 * img.at<double>(iv + j + 1, iu + i + 1);
    }
  }
}

void get_image_patch_with_mask(const cv::Mat &img, const cv::Mat &mask, double u, double v, int r, cv::Mat &img_sub) {
  int iu = u;
  int iv = v;
  double du = u - iu;
  double dv = v - iv;
  double a00 = 1 - du - dv + du * dv;
  double a01 = du - du * dv;
  double a10 = dv - du * dv;
  double a11 = du * dv;

  img_sub.create((2 * r + 1) * (2 * r + 1), 1, CV_64F);
  int num = 0;
  for (int j = -r; j <= r; ++j) {
    for (int i = -r; i <= r; ++i) {
      if (mask.at<double>(j + r, i + r) >= 1e-6) {
        img_sub.at<double>(num, 0) =
            a00 * img.at<double>(iv + j, iu + i) + a01 * img.at<double>(iv + j, iu + i + 1) +
                a10 * img.at<double>(iv + j + 1, iu + i) + a11 * img.at<double>(iv + j + 1, iu + i + 1);
        ++num;
      }
    }
  }
  img_sub.resize(num);
}

}

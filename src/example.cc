#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include "libcbdetect/chessboards_from_corners.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_chessboards.h"
#include "libcbdetect/plot_corners.h"

using namespace std::chrono;

int main(int argc, char *argv[]) {
  cbdetect::Corner corners;
  std::vector<std::vector<std::vector<int>>> chessboards;
  cbdetect::Params params;
  params.show_processing = true;
  params.detct_mode = cbdetect::HessianResponse;
  params.score_thr = 0.01;
  params.hessian_thr = 0.1;
  params.radius = {3, 5, 7};
  params.norm = false;
  params.polynomial_fit = true;
  params.polynomial_fit_half_kernel_size = 3;
  params.norm_half_kernel_size = 101;

  cv::Mat img = cv::imread("../../example_data/e2.png", cv::IMREAD_COLOR);
  auto t1 = high_resolution_clock::now();
  cbdetect::find_corners(img, corners, params);
  auto t2 = high_resolution_clock::now();
  cbdetect::plot_corners(img, corners);
  auto t3 = high_resolution_clock::now();
  cbdetect::chessboards_from_corners(corners, chessboards);
  auto t4 = high_resolution_clock::now();
  cbdetect::plot_chessboards(img, corners, chessboards);
  printf("Total took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0
      + duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  return 0;
}

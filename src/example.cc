#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include "libcbdetect/config.h"
#include "libcbdetect/chessboards_from_corners.h"
#include "libcbdetect/deltilles_from_corners.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_chessboards.h"
#include "libcbdetect/plot_deltilles.h"
#include "libcbdetect/plot_corners.h"

using namespace std::chrono;

void chessboard() {
  cbdetect::Corner corners;
  std::vector<std::vector<std::vector<int>>> chessboards;
  cbdetect::Params params;

  cv::Mat img = cv::imread("../../example_data/e2.png", cv::IMREAD_COLOR);
  auto t1 = high_resolution_clock::now();
  cbdetect::find_corners(img, corners, params);
  auto t2 = high_resolution_clock::now();
  cbdetect::plot_corners(img, corners);
  auto t3 = high_resolution_clock::now();
  cbdetect::chessboards_from_corners(corners, chessboards);
  auto t4 = high_resolution_clock::now();

  printf("Find corners took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0);
  printf("Find chessboard took: %.3f ms\n", duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  printf("Total took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0
      + duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  cbdetect::plot_chessboards(img, corners, chessboards);
}

void deltille() {
  cbdetect::Corner corners;
  std::vector<cbdetect::Deltille> deltilles;
  cbdetect::Params params;
  params.corner_type = cbdetect::MonkeySaddlePoint;

  cv::Mat img = cv::imread("../../example_data/e6.png", cv::IMREAD_COLOR);
  auto t1 = high_resolution_clock::now();
  cbdetect::find_corners(img, corners, params);
  auto t2 = high_resolution_clock::now();
  cbdetect::plot_corners(img, corners);
  auto t3 = high_resolution_clock::now();
  cbdetect::deltilles_from_corners(img, corners, deltilles, params);
  auto t4 = high_resolution_clock::now();

  printf("Find corners took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0);
  printf("Find deltille took: %.3f ms\n", duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  printf("Total took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0
      + duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  cbdetect::plot_deltilles(img, corners, deltilles);
}

int main(int argc, char *argv[]) {
  printf("detect chessboard...\n");
  chessboard();
  printf("\n");
  printf("detect deltille...\n");
  deltille();
  return 0;
}

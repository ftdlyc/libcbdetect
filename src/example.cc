#include <opencv2/opencv.hpp>
#include "libcbdetect/chessboards_from_corners.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_chessboards.h"
#include "libcbdetect/plot_corners.h"

int main(int argc, char *argv[]) {
  cbdetect::Corner corners;
  std::vector<std::vector<std::vector<int>>> chessboards;

  cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cbdetect::find_corners(img, corners, {true, false, 0.01});
  cbdetect::plot_corners(img, corners);
  cbdetect::chessboards_from_corners(corners, chessboards);
  cbdetect::plot_chessboards(img, corners, chessboards);
  return 0;
}

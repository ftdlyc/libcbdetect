// c++ version by ftdlyc

#include "grow_deltille.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "grow_chessboard.h"

namespace cbdetect {

bool add_deltille_board(Deltille &deltille, int board_type) {
  int rows = deltille.idx.size(), cols = deltille.idx[0].size();
  bool add_board = false;

  // top/left/bottom//right
  switch (board_type) {
    case 0: {
      for (int i = 0; i < cols; ++i) {
        if (deltille.idx[0][i] != -2 && deltille.idx[0][i] != -1) {
          add_board = true;
          break;
        }
      }
      if (add_board) {
        std::vector<int> idx(cols, -1);
        std::vector<std::vector<double>> energy(cols, std::vector<double>(3, DBL_MAX));
        deltille.idx.insert(deltille.idx.begin(), idx);
        deltille.energy.insert(deltille.energy.begin(), energy);
      }
      break;
    }
    case 1: {
      for (int i = 0; i < rows; ++i) {
        if (deltille.idx[i][0] != -2 && deltille.idx[i][0] != -1) {
          add_board = true;
          break;
        }
      }
      if (add_board) {
        for (int i = 0; i < rows; ++i) {
          deltille.idx[i].insert(deltille.idx[i].begin(), -1);
          deltille.energy[i].insert(deltille.energy[i].begin(), std::vector<double>(3, DBL_MAX));
        }
      }
      break;
    }
    case 2: {
      for (int i = 0; i < cols; ++i) {
        if (deltille.idx[rows - 1][i] != -2 && deltille.idx[rows - 1][i] != -1) {
          add_board = true;
          break;
        }
      }
      if (add_board) {
        std::vector<int> idx(cols, -1);
        std::vector<std::vector<double>> energy(cols, std::vector<double>(3, DBL_MAX));
        deltille.idx.emplace_back(idx);
        deltille.energy.emplace_back(energy);
      }
      break;
    }
    case 3: {
      for (int i = 0; i < rows; ++i) {
        if (deltille.idx[i][cols - 1] != -2 && deltille.idx[i][cols - 1] != -1) {
          add_board = true;
          break;
        }
      }
      if (add_board) {
        for (int i = 0; i < rows; ++i) {
          deltille.idx[i].emplace_back(-1);
          deltille.energy[i].emplace_back(std::vector<double>(3, DBL_MAX));
        }
      }
      break;
    }
    default:break;
  }

  return add_board;
}

std::vector<int> predict_deltille_corner(const Corner &corners,
                                         std::vector<int> &used,
                                         std::vector<int> p1,
                                         std::vector<int> p2,
                                         std::vector<int> p3) {
  std::vector<cv::Point2d> pred = predict_corners(corners, p1, p2, p3);
  std::vector<int> pred_idx(pred.size(), -2);

  // build distance matrix
  std::vector<std::vector<double>> D(pred.size(), std::vector<double>(corners.p.size(), DBL_MAX));
  for (int i = 0; i < pred.size(); ++i) {
    cv::Point2d w = pred[i] - corners.p[p3[i]];
    for (int j = 0; j < corners.p.size(); ++j) {
      if (used[j] == 1) { continue; }
      cv::Point2d v_tmp = corners.p[j] - corners.p[p3[i]];
      cv::Point2d v(v_tmp.dot(w), v_tmp.dot(cv::Point2d(w.y, -w.x)));
      v = v / (cv::norm(w) * cv::norm(w));
      double d1 = std::atan2(v.y, v.x);
      double d2 = (1 - cv::norm(v));
      D[i][j] = std::sqrt(std::abs(d1) + d2 * d2 * d2 * d2);
    }
  }

  // search for closest corners
  for (int i = 0; i < pred.size(); ++i) {
    double min_D = DBL_MAX;
    int min_row = 0;
    int min_col = 0;
    for (int j = 0; j < pred.size(); ++j) {
      int min_row_2 = std::min_element(D[j].begin(), D[j].end()) - D[j].begin();
      if (D[j][min_row_2] < min_D) {
        min_D = D[j][min_row_2];
        min_row = min_row_2;
        min_col = j;
      }
    }

    // all detect corners have been used
    if (DBL_MAX - min_D < 1e-6) { break; }

    for (auto &j : D[min_col]) { j = DBL_MAX; }
    for (int j = 0; j < pred.size(); ++j) {
      D[j][min_row] = DBL_MAX;
    }
    pred_idx[min_col] = min_row;
    used[min_row] = 1;
  }

  return pred_idx;
}

bool grow_deltille(const Corner &corners, std::vector<int> &used, Deltille &deltille,
                   std::vector<cv::Point2i> &proposal, int board_type) {
  // return immediately, if there do not exist any chessboards
  if (deltille.idx.empty()) { return false; }

  // add proposal top/left/bottom/right
  if (!add_deltille_board(deltille, board_type)) { return false; }
  int cols = deltille.idx[0].size();
  int rows = deltille.idx.size();
  std::vector<int> idx, p1, p2, p3;
  switch (board_type) {
    case 0: {
      for (int i = 0; i < cols; ++i) {
        int idx1 = deltille.idx[3][i];
        int idx2 = deltille.idx[2][i];
        int idx3 = deltille.idx[1][i];
        if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
        p1.emplace_back(idx1);
        p2.emplace_back(idx2);
        p3.emplace_back(idx3);
        proposal.emplace_back(cv::Point2i(i, 0));
      }
      for (int i = rows - 4; i > 0; --i) {
        for (int j = 0; j < cols; ++j) {
          if (deltille.idx[i][j] != -1) { continue; }
          int idx1 = deltille.idx[i + 3][j];
          int idx2 = deltille.idx[i + 2][j];
          int idx3 = deltille.idx[i + 1][j];
          if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 1: {
      for (int i = 0; i < rows; ++i) {
        int idx1 = deltille.idx[i][3];
        int idx2 = deltille.idx[i][2];
        int idx3 = deltille.idx[i][1];
        if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
        p1.emplace_back(idx1);
        p2.emplace_back(idx2);
        p3.emplace_back(idx3);
        proposal.emplace_back(cv::Point2i(0, i));
      }
      for (int i = 0; i < rows; ++i) {
        for (int j = cols - 4; j > 0; --j) {
          if (deltille.idx[i][j] != -1) { continue; }
          int idx1 = deltille.idx[i][j + 3];
          int idx2 = deltille.idx[i][j + 2];
          int idx3 = deltille.idx[i][j + 1];
          if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 2: {
      for (int i = 0; i < cols; ++i) {
        int idx1 = deltille.idx[rows - 4][i];
        int idx2 = deltille.idx[rows - 3][i];
        int idx3 = deltille.idx[rows - 2][i];
        if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
        p1.emplace_back(idx1);
        p2.emplace_back(idx2);
        p3.emplace_back(idx3);
        proposal.emplace_back(cv::Point2i(i, rows - 1));
      }
      for (int i = 3; i < rows - 1; ++i) {
        for (int j = 0; j < cols; ++j) {
          if (deltille.idx[i][j] != -1) { continue; }
          int idx1 = deltille.idx[i - 3][j];
          int idx2 = deltille.idx[i - 2][j];
          int idx3 = deltille.idx[i - 1][j];
          if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 3: {
      for (int i = 0; i < rows; ++i) {
        int idx1 = deltille.idx[i][cols - 4];
        int idx2 = deltille.idx[i][cols - 3];
        int idx3 = deltille.idx[i][cols - 2];
        if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
        p1.emplace_back(idx1);
        p2.emplace_back(idx2);
        p3.emplace_back(idx3);
        proposal.emplace_back(cv::Point2i(cols - 1, i));
      }
      for (int i = 0; i < rows; ++i) {
        for (int j = 3; j < cols - 1; ++j) {
          if (deltille.idx[i][j] != -1) { continue; }
          int idx1 = deltille.idx[i][j - 1];
          int idx2 = deltille.idx[i][j - 2];
          int idx3 = deltille.idx[i][j - 3];
          if (idx1 < 0 || idx2 < 0 || idx3 < 0) { continue; }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    default:break;
  }

  // predict corners
  std::vector<int> pred = predict_deltille_corner(corners, used, p1, p2, p3);
  deltille.num += proposal.size();
  for (int i = 0; i < proposal.size(); ++i) {
    if (pred[i] == -2) { --deltille.num; }
    deltille.idx[proposal[i].y][proposal[i].x] = pred[i];
  }
  return true;
}

}

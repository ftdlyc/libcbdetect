// c++ version by ftdlyc

#include "filter_deltille.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "deltille_energy.h"

namespace cbdetect {

void filter_deltille(const Corner &corners, std::vector<int> &used, Deltille &deltille,
                     std::vector<cv::Point2i> &proposal, double &energy) {
  while (!proposal.empty()) {
    cv::Point3i maxE_pos = deltille_energy(corners, deltille);
    double p_energy = deltille.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if (p_energy <= energy) {
      energy = p_energy;
      break;
    }

    cv::Point2i p[3];
    p[0] = {maxE_pos.x, maxE_pos.y};
    switch (maxE_pos.z) {
      case 0: {
        p[1] = {maxE_pos.x + 1, maxE_pos.y};
        p[2] = {maxE_pos.x + 2, maxE_pos.y};
        break;
      }
      case 1: {
        p[1] = {maxE_pos.x + 1, maxE_pos.y + 1};
        p[2] = {maxE_pos.x + 2, maxE_pos.y + 2};
        break;
      }
      case 2: {
        p[1] = {maxE_pos.x, maxE_pos.y + 1};
        p[2] = {maxE_pos.x, maxE_pos.y + 2};
        break;
      }
      default:break;
    }
    double minE_wrong[3];
    minE_wrong[0] = std::min(std::min(deltille.energy[p[0].y][p[0].x][0], deltille.energy[p[0].y][p[0].x][1]),
                             deltille.energy[p[0].y][p[0].x][2]);
    minE_wrong[1] = std::min(std::min(deltille.energy[p[1].y][p[1].x][0], deltille.energy[p[1].y][p[1].x][1]),
                             deltille.energy[p[1].y][p[1].x][2]);
    minE_wrong[2] = std::min(std::min(deltille.energy[p[2].y][p[2].x][0], deltille.energy[p[2].y][p[2].x][1]),
                             deltille.energy[p[2].y][p[2].x][2]);
    double minE = DBL_MIN;
    decltype(proposal.begin()) iter;
    for (auto it = proposal.begin(); it < proposal.end(); ++it) {
      if (it->x == p[0].x && it->y == p[0].y) {
        if (minE_wrong[0] > minE) {
          minE = minE_wrong[0];
        }
        maxE_pos.x = it->x;
        maxE_pos.y = it->y;
        iter = it;
      }
      if (it->x == p[1].x && it->y == p[1].y) {
        if (minE_wrong[1] > minE) {
          minE = minE_wrong[1];
        }
        maxE_pos.x = it->x;
        maxE_pos.y = it->y;
        iter = it;
      }
      if (it->x == p[2].x && it->y == p[2].y) {
        if (minE_wrong[2] > minE) {
          minE = minE_wrong[2];
        }
        maxE_pos.x = it->x;
        maxE_pos.y = it->y;
        iter = it;
      }
    }

    proposal.erase(iter);
    used[deltille.idx[maxE_pos.y][maxE_pos.x]] = 0;
    deltille.idx[maxE_pos.y][maxE_pos.x] = -2;
    --deltille.num;
  }
}

}


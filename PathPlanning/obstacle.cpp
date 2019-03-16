/*
 * node.cpp
 *
 *   Author: Alexis Leautier
 */

#include "obstacle.hpp"

namespace RRT {

Obstacle::Obstacle(const double vertices[4][2]) {

  // Using the 2D coordinates of the four vertices, build H and b
  double x1, x2, y1, y2;
  for (int i = 0; i < 3; i++) {
    x1 = vertices[i][0];
    y1 = vertices[i][1];
    x2 = vertices[i + 1][0];
    y2 = vertices[i + 1][1];
    H_[i][0] = y1 - y2;
    H_[i][1] = x2 - x1;
    b_[i] = x1 * y2 - x2 * y1;
  }
  x1 = vertices[3][0];
  y1 = vertices[3][1];
  x2 = vertices[0][0];
  y2 = vertices[0][1];
  H_[3][0] = y1 - y2;
  H_[3][1] = x2 - x1;
  b_[3] = x1 * y2 - x2 * y1;
}

} // namespace RRT

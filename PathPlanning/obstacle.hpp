/*
 * obstacle.hpp
 *
 *   Author: Alexis Leautier
 */

#ifndef PATH_PLANNING_OBSTACLE_HPP_
#define PATH_PLANNING_OBSTACLE_HPP_

namespace RRT {

/* OBSTACLE:
 * Define the obstacle class. Each instance is initialized by providing the
 * coordinates of 4 points which will become the corners of a rectangle shaped
 * obstacle. Each obstacle object inherits a matrix H and a vector b such that
 * for each point of coordinates [x y]', each line of the vector H*[x y]' + b
 * must be negative if the point is inside the obstacle. This representation
 * allows to quickly check if a node is safe.
 *
 */
class Obstacle {
public:
  // Initializer
  Obstacle(const double vertices[4][2]);

  // Reference to H_
  double &H(const int i, const int j) { return H_[i][j]; }

  // Reference to b_
  double &b(const int i) { return b_[i]; }

private:
  // model: Hx = b
  double H_[4][2];
  double b_[4];
};
} // namespace RRT

#endif // PATH_PLANNING_OBSTACLE_HPP_

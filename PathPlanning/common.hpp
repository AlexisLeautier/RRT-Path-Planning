/*
 * common.hpp
 *
 *   Author: Alexis Leautier
 */

#ifndef PATH_PLANNING_COMMON_HPP_
#define PATH_PLANNING_COMMON_HPP_

namespace RRT {

/* POINT2D
 *
 */
class Point2d {
public:
  Point2d() : x_(0.0), y_(0.0){};
  Point2d(const double x, const double y) : x_(x), y_(y){};

  // Get x by reference and value
  double &x() { return x_; }
  double x() const { return x_; }

  // Get y by reference and value
  double &y() { return y_; }
  double y() const { return y_; }

private:
  double x_ = 0.0;
  double y_ = 0.0;
};

} // namespace RRT

#endif // PATH_PLANNING_COMMON_HPP_

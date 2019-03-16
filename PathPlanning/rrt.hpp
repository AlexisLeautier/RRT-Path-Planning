/*
 * RRT.hpp
 *
 *   Author: Alexis Leautier
 *
 * This header defines the namespace RRT which contains the classes Node,
 * Tree and Obstacle. The Tree class is used along with its member functions
 * to generate and explore a domain while keeping track of the parts it has
 * already explored. During the search, each iteration creates a new instance
 * of the Node class whose coordinates and relation to other neighboring nodes
 * are accessible at any time, while making sure we are avoiding the obstacles.
 *
 */

#ifndef PATH_PLANNING_RRT_HPP_
#define PATH_PLANNING_RRT_HPP_

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <math.h>
#include <ostream>
#include <vector>

#include "common.hpp"
#include "node.hpp"
#include "obstacle.hpp"

namespace RRT {

/* TREE:
 * Define the Tree class. Each instance inherits a Node head that
 * represents the starting point of the trajectory, a set of 2D
 * coordinates representing the goal, a vector of obstacles and
 * 2D boundaries that define the domain. Finally, branchLength
 * represents the distance from one point its children in the tree.
 * A set of member functions is also defined to efficiently search
 * the tree.
 *
 */
class Tree {
public:
  // Initializer
  Tree(const Point2d startingPoint, const Point2d goal, const Point2d &xBound,
       const Point2d &yBound);

  // Search a path from head to goal while avoiding obstacles
  void SearchTrajectory(Node &TrajTail);

  // Print the coordinates of each node in the path to Trajectory.csv
  void PrintToCSV(const Node &tail);

  // Generate a set of n random obstacles
  void GenerateRandomObstacles(const int n);

private:
  // Compute the distance from point A to point B
  double Distance(const Point2d &A, const Point2d &B);

  // Check if a given point is in the neighborhood of the goal.
  // The goal is considered reached if a node is located in a disk of
  // radius "tolerance" around it.
  bool IsAchieved(const Point2d &point, const double tolerance);

  // Check if a point is safe with respect to the defined obstacles
  bool IsSafe(const Point2d &point);

  // Return in a pair the minimum distance to a given point from all points
  // in the current branch defined by root and the node where this distance
  // is achieved.
  std::pair<Node *, double>
  SearchBranch(Node *const root, const Point2d &point,
               const std::pair<Node *, double> &currMinPair);

  Node head_;
  Point2d goal_;
  std::vector<Obstacle> obstacles_;
  Point2d xBound_;
  Point2d yBound_;
  double branchLength_ = 0.0;
};

} // namespace RRT

#endif // PATH_PLANNING_RRT_HPP_

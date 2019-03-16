/*
 * node.hpp
 *
 *   Author: Alexis Leautier
 */

#ifndef PATH_PLANNING_NODE_HPP_
#define PATH_PLANNING_NODE_HPP_

#include <vector>

#include "common.hpp"

namespace RRT {

/* NODE:
 * Define the Node class. Each Node is a set of x and y coordinates with a
 * redundant representation that is linked to one parent node and to a set
 * of children. A vector representation is used for the set of children as
 * its dynamic nature handles the fact that we cannot predict the number of
 * children that a node will have.
 *
 */
class Node {
public:
  // Initializer
  Node();
  Node(const Point2d &coordinates) : coordinates_(coordinates){};

  // Assign x, y and coordinates
  void SetXY(const double x, const double y);

  // Add the node to the tree by assigning its parent and registering it
  // as its parent's child
  void SetParent(Node *const parent);
  Node *parent() const { return parent_; }

  std::vector<Node *> children() const { return children_; }
  std::vector<Node *> &children() { return children_; }

  Point2d coordinates() const { return coordinates_; }
  Point2d &coordinates() { return coordinates_; }

private:
  Node *parent_ = nullptr;
  std::vector<Node *> children_;
  Point2d coordinates_;
};
} // namespace RRT

#endif // PATH_PLANNING_NODE_HPP_

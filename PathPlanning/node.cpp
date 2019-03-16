/*
 * node.cpp
 *
 *   Author: Alexis Leautier
 */

#include "node.hpp"

namespace RRT {

Node::Node() {

  parent_ = nullptr;
  coordinates_.x() = 0.0;
  coordinates_.y() = 0.0;
}

// Assign x, y and coordinates
void Node::SetXY(const double x, const double y) {

  coordinates_.x() = x;
  coordinates_.y() = y;
  return;
}

// Add the node to the tree by assigning its parent and registering it
// as its parent's child
void Node::SetParent(Node *const parent) {
  if (parent) {
    parent_ = parent;
    parent->children_.push_back(this);
  }
  return;
}

} // namespace RRT

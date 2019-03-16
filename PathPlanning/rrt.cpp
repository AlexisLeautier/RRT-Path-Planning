/*
 * RRT.cpp
 *
 *   Author: Alexis Leautier
 *
 * In this file are defined the member functions for the Obstacle, Node
 * and Tree classes in the namespace RRT.
 */

#include "rrt.hpp"
#include "node.hpp"
#include "obstacle.hpp"

namespace RRT {

Tree::Tree(const Point2d startingPoint, const Point2d goal,
           const Point2d &xBound, const Point2d &yBound)
    : head_(startingPoint), goal_(goal), xBound_(xBound), yBound_(yBound) {

  branchLength_ =
      std::min(xBound_.y() - xBound_.x(), yBound_.y() - yBound_.x()) / 100;
}

// Compute the distance from point A to point B
double Tree::Distance(const Point2d &A, const Point2d &B) {

  double d;
  d = sqrt((A.x() - B.x()) * (A.x() - B.x()) +
           (A.y() - B.y()) * (A.y() - B.y()));
  return d;
}

// Check if a given point is in the neighborhood of the goal.
// The goal is considered reached if a node is located in a disk of
// radius "tolerance" around it.
bool Tree::IsAchieved(const Point2d &point, const double tolerance) {

  if (Distance(goal_, point) < tolerance) {
    return true;
  }
  return false;
}

// Check if a point is safe with respect to the defined obstacles
bool Tree::IsSafe(const Point2d &point) {

  for (unsigned int k = 0; k < obstacles_.size(); k++) {
    bool iflag(false);
    for (int i = 0; i < 4; ++i) {
      iflag = iflag or (obstacles_.at(k).H(i, 0) * point.x() +
                            obstacles_.at(k).H(i, 1) * point.y() +
                            obstacles_.at(k).b(i) >
                        0.0);
    }
    if (!iflag) {
      return false;
    }
  }
  return true;
}

// Return in a pair the minimum distance to a given point from all points
// in the current branch defined by root and the node where this distance
// is achieved.
std::pair<Node *, double>
Tree::SearchBranch(Node *const root, const Point2d &point,
                   const std::pair<Node *, double> &currMinPair) {

  Node *iNode = root;
  std::pair<Node *, double> minPair = currMinPair;
  double minDistance = minPair.second;

  // Search branch until the next fork
  while (iNode->children().size() == 1) {
    if (Distance(iNode->coordinates(), point) < minDistance) {
      minPair.first = iNode;
      minDistance = Distance(iNode->coordinates(), point);
      minPair.second = minDistance;
    }
    iNode = iNode->children().at(0);
  }

  // Search last node
  if (Distance(iNode->coordinates(), point) < minDistance) {
    minPair.first = iNode;
    minDistance = Distance(iNode->coordinates(), point);
    minPair.second = minDistance;
  }

  // Search each ramification
  if (iNode->children().size() > 1) {
    std::pair<Node *, double> newPair;
    for (unsigned int i = 0; i < iNode->children().size(); i++) {
      newPair = SearchBranch(iNode->children().at(i), point, minPair);
      if (newPair.second < minPair.second) {
        minPair = newPair;
      }
    }
  }
  return minPair;
}

// Search a path from head to goal while avoiding obstacles
void Tree::SearchTrajectory(Node &TrajTail) {

  // Initialize temporary variables
  Point2d randPoint, directionVector;
  std::pair<Node *, double> minPair;
  double tolerance = 2 * branchLength_;
  int itn(0);

  // Initialize random generator seed
  srand(time(NULL));

  // Save all node coordinates in Tree.csv
  std::ofstream outFile;
  outFile.open("Results/Tree.csv");
  outFile << "x,y\n";
  outFile << head_.coordinates().x() << "," << head_.coordinates().y() << "\n";

  // Loop until we reach 5000 iterations or exit when a path is found
  while (itn < 5000) {

    // Generate a random point
    randPoint.x() =
        ((double)rand() / (RAND_MAX)) * (xBound_.y() - xBound_.x()) +
        xBound_.x();
    randPoint.y() =
        ((double)rand() / (RAND_MAX)) * (yBound_.y() - yBound_.x()) +
        yBound_.x();

    // Search through the whole tree for the closest node
    minPair.second = Distance(head_.coordinates(), randPoint);
    minPair.first = &head_;
    minPair = SearchBranch(minPair.first, randPoint, minPair);

    // From the closest node, create a new node in the direction of the random
    // point
    directionVector.x() = (randPoint.x() - minPair.first->coordinates().x());
    directionVector.y() = (randPoint.y() - minPair.first->coordinates().y());
    directionVector.x() /= Distance(randPoint, minPair.first->coordinates());
    directionVector.y() /= Distance(randPoint, minPair.first->coordinates());
    Node *iNode = new Node;
    iNode->SetXY(
        minPair.first->coordinates().x() + directionVector.x() * branchLength_,
        minPair.first->coordinates().y() + directionVector.y() * branchLength_);

    // Verify that the new node is safe with respect to the obstacles
    if (IsSafe(iNode->coordinates())) {
      iNode->SetParent(minPair.first);
      outFile << iNode->coordinates().x() << "," << iNode->coordinates().y()
              << "\n";
    }

    // Check if the new node reached the goal
    if (IsAchieved(iNode->coordinates(), tolerance)) {
      TrajTail = *iNode;
      std::cout << "Success !\nA trajectory was found." << std::endl;
      return;
    }
    itn++;
  }

  // CLose Tree.csv
  outFile.close();

  // return head_ if a path couldn't be found
  std::cout << "Failure !\nNo trajectory could be found." << std::endl;
  TrajTail = head_;
  return;
}

// Print the coordinates of each node in the path to Trajectory.csv
void Tree::PrintToCSV(const Node &tail) {

  Node node = tail;
  std::ofstream outFile;
  outFile.open("Results/Trajectory.csv");
  outFile << "x,y\n";
  outFile << goal_.x() << "," << goal_.y() << "\n";
  while (node.parent() != NULL) {
    outFile << node.coordinates().x() << "," << node.coordinates().y() << "\n";
    node = *node.parent();
  }
  outFile.close();
  return;
}

// Generate a set of n random obstacles
void Tree::GenerateRandomObstacles(const int n) {

  srand(time(NULL));
  double obsXY[4][2];

  // Save the coordinates of the obstacles in Obstacles.csv
  std::ofstream obsFile;
  obsFile.open("Results/Obstacles.csv");
  for (int i = 0; i < n; i++) {

    // Generate coordinates
    obsXY[0][0] = ((double)rand() / (RAND_MAX)) * (xBound_.y() - xBound_.x()) +
                  xBound_.x();
    obsXY[0][1] = ((double)rand() / (RAND_MAX)) * (yBound_.y() - yBound_.x()) +
                  yBound_.x();
    obsXY[1][0] = ((double)rand() / (RAND_MAX)) *
                      (xBound_.y() - (obsXY[0][0] + branchLength_)) +
                  obsXY[0][0];
    obsXY[1][1] = obsXY[0][1];
    obsXY[2][0] = obsXY[1][0];
    obsXY[2][1] = ((double)rand() / (RAND_MAX)) *
                      (obsXY[0][1] - (yBound_.x() + branchLength_)) +
                  yBound_.x();
    obsXY[3][0] = obsXY[0][0];
    obsXY[3][1] = obsXY[2][1];
    Obstacle obs(obsXY);
    obstacles_.push_back(obs);

    // Verify that goal and head are safe from the new obstacle
    if ((!IsSafe(goal_)) or (!IsSafe(head_.coordinates()))) {
      obstacles_.pop_back();
      i--;
      continue;
    }

    // Save coordinates of the bottom left corner, width and length
    obsFile << obsXY[3][0] << "," << obsXY[3][1] << ","
            << (obsXY[1][0] - obsXY[0][0]) << "," << (obsXY[1][1] - obsXY[2][1])
            << "\n";
  }

  // Close Obstacles.csv
  obsFile.close();
  return;
}

} // namespace RRT

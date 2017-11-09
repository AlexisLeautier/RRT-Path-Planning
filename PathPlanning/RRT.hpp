/*
 * RRT.hpp
 *
 *  Created on: Oct 23, 2017
 *      Author: Alexis Leautier
 *
 * This header defines the namespace RRT which contains the classes Node,
 * Tree and Obstacle. The Tree class is used along with its member functions
 * to generate and explore a domain while keeping track of the parts it has
 * already explored. During the search, each iteration creates a new instance
 * of the Node class whose coordinates and relation to other neighboring nodes
 * are accessible at any time, while making sure we are avoiding the obstacles.
 *
 */

#include <iostream>
#include <fstream>
#include <ostream>
#include <cstdlib>
#include <vector>
#include <math.h>

namespace RRT{

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
  // model: Hx = b
  double H [4][2];
  double b [4];

  // Initializer
  Obstacle(double vertices[4][2]);
};

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

  Node *_parent;
  std::vector<Node*> _children;
  double _x;
  double _y;
  std::pair<double, double> _coordinates;

  // Initializer
  Node();

  // Assign x, y and coordinates
  void SetXY(double x, double y);

  // Add the node to the tree by assigning its parent and registering it
  // as its parent's child
  void SetParent(Node* parent);

};


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
  Node _head;
  std::pair<double, double> _goal;
  std::vector<Obstacle> _obstacles;
  std::pair<double, double> _xBound;
  std::pair<double, double> _yBound;
  double _branchLength;

  // Initializer
  Tree(double startingPoint [2], double goal [2], std::pair<double, double> xBound, std::pair<double, double> yBound);

  // Compute the distance from point A to point B
  double distance(std::pair<double, double> A, std::pair<double, double> B);

  // Check if a given point is in the neighborhood of the goal.
  // The goal is considered reached if a node is located in a disk of
  // radius "tolerance" around it.
  bool isAchieved(std::pair<double, double> point, double tolerance);

  // Check if a point is safe with respect to the defined obstacles
  bool IsSafe(std::pair<double, double> point);

  // Return in a pair the minimum distance to a given point from all points
  // in the current branch defined by root and the node where this distance
  // is achieved.
  std::pair<Node*, double> searchBranch(Node* root, std::pair<double, double> point, std::pair<Node*, double> minPair);

  // Search a path from head to goal while avoiding obstacles
  void searchTrajectory(Node &TrajTail);

  // Print the coordinates of each node in the path to Trajectory.csv
  void printToCSV(Node tail);

  // Generate a set of n random obstacles
  void GenerateRandomObstacles(int n);
};

}

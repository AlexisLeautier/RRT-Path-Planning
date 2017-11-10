/*
 * RRT.cpp
 *
 *   Author: Alexis Leautier
 *
 * In this file are defined the member functions for the Obstacle, Node 
 * and Tree classes in the namespace RRT.
 */

#include "RRT.hpp"

using namespace RRT;

///////////////////////////////////////////////////////////////////////
// OBSTACLE:
Obstacle::Obstacle(double vertices[4][2]) {

  // Using the 2D coordinates of the four vertices, build H and b
  double x1, x2, y1, y2;
  for (int i=0; i<3; i++){
    x1 = vertices[i][0];
    y1 = vertices[i][1];
    x2 = vertices[i+1][0];
    y2 = vertices[i+1][1];
    H[i][0] = y1 - y2;
    H[i][1] = x2 - x1;
    b[i] = x1 * y2 - x2 * y1;
  }
  x1 = vertices[3][0];
  y1 = vertices[3][1];
  x2 = vertices[0][0];
  y2 = vertices[0][1];
  H[3][0] = y1 - y2;
  H[3][1] = x2 - x1;
  b[3] = x1 * y2 - x2 * y1;

}

///////////////////////////////////////////////////////////////////////
// NODE:
Node::Node(){

  _parent = NULL;
  _x = 0.0;
  _y = 0.0;

}

// Assign x, y and coordinates
void Node::SetXY(double x, double y) {

  _x = x;
  _y = y;
  _coordinates.first = x;
  _coordinates.second = y;
  return;

}

// Add the node to the tree by assigning its parent and registering it
// as its parent's child
void Node::SetParent(Node* parent){

  if (parent){
    _parent = parent;
    parent->_children.push_back(this);
  }
  return;

}

///////////////////////////////////////////////////////////////////////
// TREE:
Tree::Tree(double startingPoint [2], double goal [2], std::pair<double, double> xBound, std::pair<double, double> yBound) {

  _head._x  = startingPoint[0];
  _head._y = startingPoint[1];
  _goal.first = goal[0];
  _goal.second = goal[1];
  _xBound = xBound;
  _yBound = yBound;
  _branchLength = std::min(_xBound.second - _xBound.first,
                            _yBound.second - _yBound.first) / 100;

}

// Compute the distance from point A to point B
double Tree::distance(std::pair<double, double> A, std::pair<double, double> B){

  double d;
  d = sqrt((A.first - B.first) * (A.first - B.first) +
           (A.second - B.second) * (A.second - B.second));
  return d;

}

// Check if a given point is in the neighborhood of the goal.
// The goal is considered reached if a node is located in a disk of
// radius "tolerance" around it.
bool Tree::isAchieved(std::pair<double, double> point, double tolerance) {

  if (distance(_goal, point) < tolerance){
    return true;
  }
  return false;

}

// Check if a point is safe with respect to the defined obstacles
bool Tree::IsSafe(std::pair<double, double> point) {

  for (unsigned int k=0; k<_obstacles.size(); k++){
    bool iflag(false);
    for (int i=0; i<4; ++i){
      iflag = iflag or (_obstacles[k].H[i][0]*point.first + _obstacles[k].H[i][1]*point.second  + _obstacles[k].b[i] > 0.0);
    }
    if(!iflag){ return false;}
  }
  return true;

}

// Return in a pair the minimum distance to a given point from all points
// in the current branch defined by root and the node where this distance
// is achieved.
std::pair<Node*, double> Tree::searchBranch(Node* root, std::pair<double, double> point, std::pair<Node*, double> minPair) {

  Node* iNode = root;
  double minDistance = minPair.second;

  // Search branch until the next fork
  while (iNode->_children.size() == 1){
    if(distance(iNode->_coordinates, point) < minDistance){
      minPair.first = iNode;
      minDistance = distance(iNode->_coordinates, point);
      minPair.second = minDistance;
    }
    iNode = iNode->_children[0];
  }

  // Search last node
  if(distance(iNode->_coordinates, point) < minDistance){
    minPair.first = iNode;
    minDistance = distance(iNode->_coordinates, point);
    minPair.second = minDistance;
  }

  // Search each ramification
  if (iNode->_children.size() > 1){
    std::pair<Node*, double> newPair;
    for (unsigned int i=0; i<iNode->_children.size(); i++){
      newPair = searchBranch(iNode->_children[i], point, minPair);
      if (newPair.second < minPair.second) {
        minPair = newPair;
      }
    }
  }
  return minPair;

}

// Search a path from head to goal while avoiding obstacles
void Tree::searchTrajectory(Node &TrajTail){

  // Initialize temporary variables
  std::pair<double, double> randPoint,directionVector;
  std::pair<Node*, double> minPair;
  double tolerance = 2*_branchLength;
  int itn(0);

  // Initialize random generator seed
  srand (time(NULL));

  // Save all node coordinates in Tree.csv
  std::ofstream outFile;
  outFile.open("Results/Tree.csv");
  outFile << "x,y\n";
  outFile << _head._x << "," << _head._y << "\n";

  // Loop until we reach 5000 iterations or exit when a path is found
  while (itn < 5000){

    // Generate a random point
    randPoint.first = ((double) rand() / (RAND_MAX)) * (_xBound.second - _xBound.first) + _xBound.first;
    randPoint.second = ((double) rand() / (RAND_MAX)) * (_yBound.second - _yBound.first) + _yBound.first;

    // Search through the whole tree for the closest node
    minPair.second = distance(_head._coordinates, randPoint);
    minPair.first = &_head;
    minPair = searchBranch(minPair.first, randPoint, minPair);

    // From the closest node, create a new node in the direction of the random point
    directionVector.first = (randPoint.first - minPair.first->_x);
    directionVector.second = (randPoint.second- minPair.first->_y);
    directionVector.first /= distance(randPoint, minPair.first->_coordinates);
    directionVector.second /= distance(randPoint, minPair.first->_coordinates);
    Node* iNode = new Node;
    iNode->SetXY(minPair.first->_x + directionVector.first * _branchLength,
                    minPair.first->_y+ directionVector.second * _branchLength);

    // Verify that the new node is safe with respect to the obstacles
    if (IsSafe(iNode->_coordinates)) {
      iNode->SetParent(minPair.first);
      outFile << iNode->_x << "," << iNode->_y << "\n";
    }

    // Check if the new node reached the goal
    if (isAchieved(iNode->_coordinates,tolerance)){
      TrajTail = *iNode;
      std::cout << "Success !\nA trajectory was found." << std::endl;
      return;
    }
    itn++;
  }

  // CLose Tree.csv
  outFile.close();

  // return _head if a path couldn't be found
  std::cout << "Failure !\nNo trajectory could be found." << std::endl;
  TrajTail = _head;
  return;

}

// Print the coordinates of each node in the path to Trajectory.csv
void Tree::printToCSV(Node tail){

  Node node = tail;
  std::ofstream outFile;
  outFile.open("Results/Trajectory.csv");
  outFile << "x,y\n";
  outFile << _goal.first << "," << _goal.second << "\n";
  while(node._parent != NULL){
    outFile << node._x << "," << node._y << "\n";
    node = *node._parent;
  }
  outFile.close();
  return;

}

// Generate a set of n random obstacles
void Tree::GenerateRandomObstacles(int n){

  srand (time(NULL));
  double obsXY[4][2];

  // Save the coordinates of the obstacles in Obstacles.csv
  std::ofstream obsFile;
  obsFile.open("Results/Obstacles.csv");
  for (int i=0; i<n; i++){

    // Generate coordinates
    obsXY[0][0] = ((double) rand() / (RAND_MAX)) * (_xBound.second - _xBound.first) + _xBound.first;
    obsXY[0][1] = ((double) rand() / (RAND_MAX)) * (_yBound.second - _yBound.first) + _yBound.first;
    obsXY[1][0] = ((double) rand() / (RAND_MAX)) * (_xBound.second - (obsXY[0][0] + _branchLength)) + obsXY[0][0];
    obsXY[1][1] = obsXY[0][1];
    obsXY[2][0] = obsXY[1][0];
    obsXY[2][1] = ((double) rand() / (RAND_MAX)) * (obsXY[0][1] - (_yBound.first + _branchLength)) + _yBound.first;
    obsXY[3][0] = obsXY[0][0];
    obsXY[3][1] = obsXY[2][1];
    Obstacle obs(obsXY);
    _obstacles.push_back(obs);

    // Verify that goal and head are safe from the new obstacle
    if((!IsSafe(_goal)) or (!IsSafe(_head._coordinates))){
      _obstacles.pop_back();
      i--;
      continue;
    }

    // Save coordinates of the bottom left corner, width and length
    obsFile << obsXY[3][0] << "," << obsXY[3][1] << ","
        << (obsXY[1][0]-obsXY[0][0]) << "," << (obsXY[1][1]-obsXY[2][1]) << "\n";
  }

  // Close Obstacles.csv
  obsFile.close();
  return;

}

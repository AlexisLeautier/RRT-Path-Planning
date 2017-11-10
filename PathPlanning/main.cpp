/*
 * main.cpp
 *
 *  Created on: Oct 23, 2017
 *      Author: alexis
 *
 *
 * Main Program Idea:
 *  Given an initial position and a goal, this program uses the RRT (Rapidly
 *  exploring Random Tree) technique to find a non-optimal path to get from
 *  the former to the latter, while avoiding obstacles.
 *
 * User Guide:
 *  In order to get a trajectory from one point to another, the user can define
 *  the x and y coordinates of these two points along with boundaries for the
 *  domain and obstacles on the way. Note that as the obstacles are generated
 *  randomly, it is possible for the problem to be infeasible or simply too
 *  complex.
 *  Three files are created as outputs:
 *    - Tree.csv containing the coordinates of all nodes in the tree
 *    - Trajectory.csv (if a trajectory is found only) containing the coordinates
 *    of all the nodes in the trajectory
 *    - Obstacles.csv containing the coordinates of the obstacles
 *  In order to read these files and visualize the found path, a python script
 *  was also provided as annex.
 *
 * Notes:
 *  This project is designed to simulate a vehicle searching a trajectory in a
 *  rather unpredictable but fully known environment. Each obstacle can represent
 *  a fixed obstacle (tree, house, etc.) or a moving one (pedestrian, bike, car,
 *  boat, etc.) if we run the algorithm at each time step when the positions of
 *  the obstacles are updated.
 *
 * Further improvement:
 *  - The algorithm used is the simplest form of RRT algorithms, it provides a
 *  trajectory without any certainty that the vehicle will actually be capable of
 *  following it. Using the vehicle's dynamics could be a more computationally
 *  expensive but more reliable alternative.
 *  - There is no certainty either that the provided path will be even close to an
 *  optimal solution , using an RRT* algorithm could help approach an optimal
 *  solution
 *  - The obstacles are avoided but no security margin is taken, hence no
 *  anticipation on their motion or on the size of the vehicle is made. To ensure
 *  safety, a clearance of a few units could be added around the obstacles.
 *  - At each iteration of the algorithm, the whole tree is searched which is very
 *  expensive computationally. Other existing search algorithms could be used to
 *  shorten this step.
 *
 */

#include "RRT.hpp"

using namespace RRT;


int main() {
  // User input: (x,y) coordinates for the starting point and the goal to be reached
  // The coordinates must be inside the domain boundaries defined in the next paragraph
  double startingPoint[2], goal[2];
  startingPoint[0] = 0; startingPoint[1] = 0;
  goal[0] = 75;         goal[1] = 75;
  int NumberOfObstacles = 5;

  // Description of the domain boundaries
  std::pair<double, double> xBound, yBound;
  xBound.first = 0;   yBound.first = 0;
  xBound.second = 100;  yBound.second = 100;

  // Initialize the research tree with the previous values
  Tree T(startingPoint, goal, xBound, yBound);

  // Create random obstacles on the way
  T.Tree::GenerateRandomObstacles(NumberOfObstacles);

  // Initialize the last Node of the trajectory called tail
  Node tail;

  // Search a Trajectory
  T.searchTrajectory(tail);

  // Record the trajectory in the Trajectory.csv file
  T.printToCSV(tail);

  return 0;

}

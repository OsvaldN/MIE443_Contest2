# MIE443_Contest2

## Notes
- currently not `using namespace std` anywhere
- using `#pragma once` in header files rather than `#ifndef ...` as used in contest 1
- skeleton code printing is done with `std::cout` rather than `ROS_INFO` like in C1 - consider switching to `ROS_INFO`

## TODO:
- update **include/TSP.cpp** so that `distMatrix(coords)` uses actual distance from path planning rather than euclidean distance
- figure out robot starting position and modify **src/contest2.cpp** so that this position is the first one pushed to `positions`, currently assumed to be (0.0, 0.0)
- rotate the planned path in **src/contest2.cpp** so that it starts at the starting position.
- update positions vector in **src/contest2.cpp** so they are not in the center of the boxes,  but rather the position at which the robot should be to see the box. I suggest a distance away in direction *phi* which is given in `boxes.coords[i][2]`, we should also store this angle *phi* so that the robot knows which direction to face once it gets to a position. Currently the positions are stored via `positions.push_back({boxes.coords[i][0], boxes.coords[i][1]});`
- try other TSP solvers?

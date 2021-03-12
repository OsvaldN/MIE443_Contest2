# MIE443_Contest2

## running the simulation

* Please update this as progress is made *

Open three terminals. Navigate to `catkin_ws` in each of them.

1. In terminal #1 run `roslaunch mie443_contest2 turtlebot_world.launch world:=practice`

2. In terminal #2 you will need the filepath to the map `.yaml` file, here we show the full path on Osvald's VM, change the path to match your directory structure. Run `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/<REPLACE THIS WITH YOUR USER SYSTEM NAME>/catkin_ws/src/mie443_contest2/maps/map_practice.yaml`

3. In terminal #3 run `rosrun mie443_contest2 contest2`

## Notes
- currently not `using namespace std` anywhere
- using `#pragma once` in header files rather than `#ifndef ...` as used in contest 1
- skeleton code printing is done with `std::cout` rather than `ROS_INFO` like in C1 - consider switching to `ROS_INFO`

## TODO:
- update **include/TSP.cpp** so that `distMatrix(coords)` uses actual distance from path planning rather than euclidean distance
- figure out robot starting position and modify **src/contest2.cpp** so that this position is the first one pushed to `positions`, currently assumed to be (0.0, 0.0)
- try other TSP solvers?

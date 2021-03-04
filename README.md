# MIE443_Contest2

## Notes
- currently not `using namespace std` anywhere
- using `#pragma once` in header files rather than `#ifndef ...` as used in contest 1

## TODO:
- update **include/TSP.cpp** so that `distMatrix(coords)` uses actual distance from path planning rather than euclidean distance
- figure out robot starting position and modify **src/contest2.cpp** so that this position is the first one pushed to `positions`, currently assumed to be (0.0, 0.0)
- try other TSP solvers?

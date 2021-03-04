#pragma once

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <limits>

extern float eucDist(float x1, float y1, float x2, float y2);
extern std::vector<std::vector<float>> distMatrix(std::vector<std::vector<float>> coords);
extern std::tuple<std::vector<int>, float> greedy(std::vector<std::vector<float>> dM, int start);
extern std::vector<int> bestGreedy(std::vector<std::vector<float>> dM, bool verbose = true);
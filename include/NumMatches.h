#pragma once

#include <iostream>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <TSP.h>

#endif

extern int NumMatches(cv::Mat& img_object, std::vector<cv::KeyPoint> keypoints_object, cv::Mat& descriptors_object,  cv::Mat& img_scene, std::vector<cv::KeyPoint> keypoints_scene, cv::Mat& descriptors_scene, bool visual = false);



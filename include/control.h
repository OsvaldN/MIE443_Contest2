#pragma once

#include <TSP.h>
#include <robot_pose.h>
#include <navigation.h>


extern void VelPub(float angular, float linear, ros::Publisher *vel_pub);
int rotForTime(float time, ros::Publisher *vel_pub, bool verbose = true);


#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <robot_pose.h>
#include "ros/ros.h"

#define MAX_SPINRATE (M_PI/6)
#define SPEED_LIM (0.25)
#define OBS_SPEED_LIM (0.1)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)

class Navigation {
	public:
		
		Navigation(ros::NodeHandle &n); // : robotPose(0,0,0);

		RobotPose robotPose;
		ros::Subscriber amclSub;
		ros::Publisher vel_pub;

		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		bool checkPath(float xGoal, float yGoal, float phiGoal);
		
		void VelPub(float angular, float linear);
		void localizeSpin();
};

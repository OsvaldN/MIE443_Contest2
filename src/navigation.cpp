#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

Navigation::Navigation(ros::NodeHandle &n)
: robotPose(0,0,0)
{
    /* 
    Constructor for the Navigation Class
    Subscribes and advertises to relevant nodes
    */
    amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
}

bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){
	// Set up and wait for actionClient.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
	// Set goal.
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =  xGoal;
    goal.target_pose.pose.position.y =  yGoal;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = phi.z;
    goal.target_pose.pose.orientation.w = phi.w;
    ROS_INFO("Sending goal location ...");
	// Send goal and wait for response.
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have reached the destination");
        return true;
    } else {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}

bool Navigation::checkPath(float xGoal, float yGoal, float phiGoal){
    
    ros::NodeHandle nh; // not sure why this isn't working when I try to make n a member
    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("make_plan");
    nav_msgs::GetPlan srv;

    // load current position
    srv.request.start.pose.position.x = robotPose.x;
    srv.request.start.pose.position.y = robotPose.y;
    srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.phi);

    //load fgoal position
    srv.request.goal.pose.position.x = xGoal;
    srv.request.goal.pose.position.y = yGoal;
    srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(phiGoal);
    srv.request.tolerance = .1;
    
    // execute path planning
    check_path.call(srv);

    if (srv.response.plan.poses.empty()) {
        ROS_INFO("Path plan failed, retrying");
        return false;
    }

    ROS_INFO("Path plan successful");
    return true;
}


void Navigation::VelPub(float angular, float linear){
    //Publish a velocity pair using using vel_pub
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    return;
}

void Navigation::localizeSpin(){
    //TODO
    return;
}

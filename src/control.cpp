#include <boxes.h>
#include <robot_pose.h>
#include <navigation.h>
#include <control.h>

void VelPub(float angular, float linear, ros::Publisher *vel_pub){
    //Publish a velocity pair using using vel_pub
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub->publish(vel);
    return;
}

int rotForTime(float time, ros::Publisher *vel_pub, bool verbose){
    ros::Rate loop_rate(10);
    // Rotate at maximum speed in direction of angle
    float dir = 1;
    float rotVel = M_PI/6;

    if (verbose){
        ROS_INFO("Rotating for %f seconds at vel: %f rad/s", time, rotVel);
    }
    for (int i = 0; i < 10 * time; i++) {    
        // publish to update velocity, spin to update yaw (clears velocity)
        VelPub(rotVel, 0.0, vel_pub);
        loop_rate.sleep();
        ros::spinOnce();
    }
}
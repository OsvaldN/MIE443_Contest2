#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <TSP.h>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <chrono>
#include <control.h>

// for writing in files
#include <iostream>
#include <fstream>
using namespace std;

#define VIEWDIST (0.25)
#define VIEWRANGE (0.3)
#define VIEWANGLERANGE (M_PI/4.0)
#define OutputFileName "Output_file.txt"

// Macros for the position parsing from path planner
#define X_COORD 0
#define Y_COORD 1
#define PHI 2


std::vector<float> getViewPose(float boxX, float boxY, float boxPhi, bool random = false, bool verbose = true){
    /* 
    returns position from which to view a box
    - Default: VIEWDIST away from the box, looking at it head on
    - random: robot views box from a random point defined by VIEWRANGE and VIEWANGLERANGE
     */

    float offset = VIEWDIST;

    if (random){
	offset += -(VIEWRANGE/2) + static_cast <float> (rand() / static_cast <float> (RAND_MAX/(VIEWRANGE)));
	boxPhi += -(VIEWANGLERANGE/2) + static_cast <float> (rand() / static_cast <float> (RAND_MAX/VIEWANGLERANGE));
        //offset += (static_cast <float> (rand()) % VIEWRANGE) - (VIEWRANGE / 2);
        //boxPhi += (static_cast <float> (rand()) % VIEWANGLERANGE) - (VIEWANGLERANGE / 2);
    }

    float poseX = boxX + (offset * cos(boxPhi));
    float poseY = boxY + (offset * sin(boxPhi));
    // remain within [-pi, pi] interval for yaw
    float posePhi = boxPhi - ((boxPhi>0) * M_PI) + ((boxPhi<0) * M_PI);

    if (verbose){
        ROS_INFO("ViewPose created: (%f,%f) yaw: %f", poseX, poseY, posePhi);
    }

    std::vector<float> viewPose{poseX, poseY, posePhi};
    return viewPose;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    bool verbose = true; // In Debug mode, set to true

    // Used to make the output file
    // Make and overwrite any files 
    ofstream myfile;
    myfile.open (OutputFileName);
    myfile << "This is the output file which will contain the output for Contest 2 run.\n";
    myfile.close();

    // Initialize box coordinates and templates
    // get the coordinate of the boxs and the image templates to compare with
    // boxes.coords is a 2-D vector containing the boxes coordinates (10 boxes by 3 coodinates).  boxes.coords [0][2] is the first object's orientation
    // boxes.templates is a 1-D vector containing the templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    // instantiate robot Pose object
    RobotPose robotPose(0,0,0);

    // print location estimate and rotate the 
    std::cout << "Confirming location. Initial Esimate: ";
    std::cout << "(" << robotPose.x << ", " << robotPose.y << ", " 
            << robotPose.phi << ")." << std::endl;
    //rotForTime(3, &vel_pub, true); //rotates for that number of seconds (at pi/6 rad/s)
    rotForTime(3.0, &vel_pub, verbose); // rotate for 3 seconds
    std::cout << "Confirming location. New Esimate: ";
    std::cout << "(" << robotPose.x << ", " << robotPose.y << ", " 
            << robotPose.phi << ")." << std::endl;

    // positions vector holds default "poses" (x,y,phi) to view each box. No random jiggle.
    std::vector<std::vector<float>> positions;
    //push starting position, this does not generalize to new positions
    positions.push_back({0, 0, 0});

    std::vector<float> viewPose;
    for(int i = 0; i < boxes.coords.size(); ++i) {
        if (verbose) {
            std::cout << "Box coordinates: " << std::endl;
            std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                    << boxes.coords[i][2] << std::endl;
        }

        // get "default" position to view the box
	viewPose = getViewPose(boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2], false, false);
        // store "default" positions in the order they appear in the boxes instance (with home at index 0)
	positions.push_back(viewPose);
    }

    // dM holds a matrix of euclidean distances between positions
    std::vector<std::vector<float>> dM = distMatrix(positions);
    // path is vector of indices in positions that represents the planned route
    // the route aleays starts and ends at positions[0], the "home"
    std::vector<int> path = bestGreedy(dM);

    // show planned path
    std::cout << "planned path: [";
    for (int i=0;i<path.size();i++){
        std::cout << " " << path[i];
    }
    std::cout << "]" << std::endl;

    // Initiate a counter to iterate through the paths
    int path_counter = 1; // Start at index 1 (and not index 0) for the path array iteration to skip the starting home location.
    bool navigation_ret = false; // Navigation error handler
    bool path_reattempt = false; // Navigation re_attempt flag

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Create a timer in Debug Mode to time the program execution
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Execute strategy.
    
    while(ros::ok()) {

        // Loop through the paths until we reach the end of the path array
        if (path_counter > path.size()-1){
            break;
        }

        ros::spinOnce();

        // Navigation code here
        if (verbose) {
            ROS_INFO("DEBUG: Initiating move to path index: %d", path_counter);
        }

	    navigation_ret = Navigation::moveToGoal(positions[path[path_counter]][X_COORD], positions[path[path_counter]][Y_COORD], positions[path[path_counter]][PHI]);
        path_reattempt = false;

        // If move_base fails to move to the target location, enter error handling code block
        while (!navigation_ret) {
            if (verbose) {
                ROS_INFO("Error handling Navigation::move_goal. Starting from home position, and re-attempting to move to position index: %d", path_counter);
            }

            if (path_reattempt) { // If the path has already been attempted, than skip to avoid stuck in while loop.
                break;
            }

            // Step 1: Move robot to the home starting position - In a position that it has successfully moved from already
	        Navigation::moveToGoal(positions[path[0]][X_COORD], positions[path[0]][Y_COORD], positions[path[0]][PHI]);
            // Step 2: Move to the problem coordinate from the home position.
	        navigation_ret = Navigation::moveToGoal(positions[path[path_counter]][X_COORD], positions[path[path_counter]][Y_COORD], positions[path[path_counter]][PHI]);
            path_reattempt = true;
        }

        if (verbose) {
            ROS_INFO("DEBUG: Finished moving to path index: %d", path_counter);
        }
        path_counter += 1; // The path_counter will iterate through the path array that was generated from TSP path planning algorithm

        /** ***** NOTE: IMAGE DETECTION FUNCTION CALL SHOULD GO HERE *****
        At this point, the robot has successfully reached the target location. 
        You can perform image detection at this point, before allowing the robot to move to the next location.
        **/
        ros::spinOnce();
        int TemplateID = imagePipeline.getTemplateID(boxes, false, true);

        if (TemplateID > -1){
            myfile.open(OutputFileName, std::ios_base::app); // append instead of overwrite
            myfile << "The tag image at location: " << "XXX " << "is: Tag_" << TemplateID << "\n";
            myfile.close();
        }
        else{
            myfile.open(OutputFileName, std::ios_base::app); // append instead of overwrite
            myfile << "The tag image at location: " << "XXX " << "is: blank \n";
            myfile.close();
        }

        ros::Duration(0.01).sleep();
    }

    // Print out the elapsed program execution time
    if (verbose) {
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Program exeuction took this many seconds: " << secondsElapsed << '\n';
    }

    return 0;
}

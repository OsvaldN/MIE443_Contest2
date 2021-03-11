#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <TSP.h>

#include <vector>
#include <math.h>
#include <stdlib.h>

#define VIEWDIST (0.25)
#define VIEWRANGE (0.3)
#define VIEWANGLERANGE (M_PI/4.0)


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

    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    // positions vector holds default "poses" (x,y,phi) to view each box. No random jiggle.
    std::vector<std::vector<float>> positions;
    //push starting position, this does not generalize to new positions
    positions.push_back({0, 0, 0});

    std::vector<float> viewPose;
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;

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

    // instantiate a Navigation object
    Navigation nav(n);
    
    // prototype moving alg, move to nav class when working
    std::vector<float> randPose;
    for (int i=0;i<path.size();i++){
	
	nav.moveToGoal(positions[i][0], positions[i][1], positions[i][2]);

	/*
        // try to reach original target pose
        if (nav.checkPath(positions[i][0], positions[i][1], positions[i][2])){
            // move to original target
            nav.moveToGoal(positions[i][0], positions[i][1], positions[i][2]);
        }
        else {
            // get a random modification to original viewpose
            randPose = getViewPose(boxes.coords[path[i]][0], boxes.coords[path[i]][1], boxes.coords[path[i]][2], true, true);
            
	    // This while loop continuosly fails
	    while (!(nav.checkPath(randPose[0], randPose[1], randPose[2]))){
                // checkPath failed, try new pose
                randPose = getViewPose(boxes.coords[path[i]][0], boxes.coords[path[i]][1], boxes.coords[path[i]][2], true, true);
            }
	    
        } */

        // Robot is now looking at box represented by path[i]
        
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}

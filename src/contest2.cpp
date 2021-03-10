#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <TSP.h>
#include <vector>
#include <math.h>

#define VIEWDIST (0.5)

std::vector<float> getViewPose(float boxX, float boxY, float boxPhi){
    // returns position from which to view a box
    // TODO: check if this pose is possible, if not adjust somehow

    float poseX = boxX + (VIEWDIST * cos(boxPhi));
    float poseY = boxY + (VIEWDIST * sin(boxPhi));
    // remain within [-pi, pi] interval for yaw
    float posePhi = boxPhi - ((boxPhi>0) * M_PI) + ((boxPhi<0) * M_PI);
    
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

    std::vector<std::vector<float>> positions;
    //push starting position, I am assuming it is 0,0 from robotPose line?
    positions.push_back({0, 0, 0});

    std::vector<float> viewPose;
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;

        viewPose = getViewPose(boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
        positions.push_back(viewPose);
    }

    std::vector<std::vector<float>> dM = distMatrix(positions);
    std::vector<int> path = bestGreedy(dM);
    
    // show planned path
    std::cout << "planned path: [";
    for (int i=0;i<path.size();i++){
        std::cout << " " << path[i];
    }
    std::cout << "]" << std::endl;

    Navigation nav(n);
    
    // prototype moving alg, move to nav class when working
    for (int i=0;i<path.size();i++){
        // move to next goal
        nav.moveToGoal(positions[i][0], positions[i][1], positions[i][2]);
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

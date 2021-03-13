#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <TSP.h>
#include <NumMatches.h>
#include <vector>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    //update the robots positon in (x,y,yaw)
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    // get the coordinate of the boxs and the image templates to compare with
    // boxes.coords is a 2-D vector containing the boxes coordinates (10 boxes by 3 coodinates).  boxes.coords [0][2] is the first object's orientation
    // boxes.templates is a 1-D vector containing the templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    std::vector<std::vector<float>> positions;
    //push starting position, I am assuming it is 0,0 from robotPose line?
    positions.push_back({0, 0});

    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;

        //TODO: get desired robot position to view box, currently using box centre
        positions.push_back({boxes.coords[i][0], boxes.coords[i][1]});
    }

    std::vector<std::vector<float>> dM = distMatrix(positions);
    std::vector<int> path = bestGreedy(dM); // determine the path to take
    
    // show planned path
    std::cout << "planned path: [";
    for (int i=0;i<path.size();i++){
        std::cout << " " << path[i];
    }
    std::cout << "]" << std::endl;

    // Initialize image object and subscriber.
    // return an image from the Kinect sensor
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    
    while(ros::ok()) {
        ros::spinOnce();

        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        imagePipeline.getTemplateID(boxes, true, true); // Match the image from image callback with one from the templates (needs to be coded in imagePipepline.cpp file)
        ros::Duration(0.01).sleep();
    }
    return 0;
}

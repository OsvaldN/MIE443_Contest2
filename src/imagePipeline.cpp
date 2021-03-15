#include <imagePipeline.h>

#define minHessian (350) // default vale is 400 (high is better matches but less points)
#define minThreshMatches (20) //to detect whether an image was found

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

//get the image of from the kinect sensor
void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes, bool visual, bool verbose) { // a copy of boxes is feed into this function
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        // call function to find the best match image

        // note - img is a image from the scene, and boxes.template[x] is an image form the template list (format is cv::MAT)

        // convert image to grey scale
        cv::Mat grey_img;
        cv::cvtColor(img, grey_img, CV_BGR2GRAY);

        // search through the templates and see which is the best match

        int max_matches = 0;
        int next_max_matches = 0; // the the 2nd most matches
        int confidence = 0; //save the confidence level of the choosen template

        for (int i = 0; i < boxes.templates.size(); i++){

            int good_matches = NumMatches(boxes.templates[i], grey_img, minHessian, false);

            if (good_matches > max_matches){
                template_id = i;
                next_max_matches = max_matches;
                max_matches = good_matches;
            }
            else if (good_matches > next_max_matches){
                next_max_matches = good_matches;
            }

            if (verbose){
                ROS_INFO("The number of good matches is: %i for tag number: %i", good_matches, i+1);
            }

        }

        confidence = (-500000/((max_matches*minHessian) + 14286)) + 35 + (-1500/((max_matches - next_max_matches) + 23.08)) + 65;

        if (max_matches > minThreshMatches)
        {
            if (visual)
            {
                max_matches = NumMatches(boxes.templates[template_id], grey_img, minHessian, true);
            }
            template_id = template_id + 1;

            if (verbose)
            {
                ROS_INFO("The best match template_id is %i and the confidence level is %i percent", template_id, confidence);
            }
        }
        else
        {
            template_id = -1;

            if (verbose)
            {
                ROS_INFO("No image was found");
            }
        }

        cv::waitKey(); // STOPS THE PROGRAM !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
    }  
    return template_id;
}

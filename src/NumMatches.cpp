#include <NumMatches.h>

#define HTHRESH (150)
#define minDistTHRESH (0.1)
#define SKEWFACTOR (4)

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

int NumMatches(cv::Mat& img_object, cv::Mat& img_scene, int minHessian, bool visual) {

    // TO DO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // play around with the minHessian to see what produces good values.
    // Maybe a function could be used if we can find some correlation with minHessian to something else eg. distance to image or overall brightness or something

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f; // BY DEFAULT THIS IS SET TO 0.75f !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    //-- Localize the object
    // Save the keypoints for the object and scene into seperate vectors
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    std::vector<Point2f> scene_ideal; // obj*Homography = scene_ideal



    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    

    if (!obj.empty() && !scene.empty()) { // to prevent this error -> OpenCV Error: Bad argument (The input arrays should be 2D or 3D point sets) in findHomography

        Mat H = findHomography( obj, scene, RANSAC );
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = Point2f(0, 0);
        obj_corners[1] = Point2f( (float)img_object.cols, 0 );
        obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
        obj_corners[3] = Point2f( 0, (float)img_object.rows );
        std::vector<Point2f> scene_corners(4);

        if (! H.empty()) // added due to - OpenCV Error: Assertion failed (scn + 1 == m.cols) in perspectiveTransform
        {   
            perspectiveTransform( obj_corners, scene_corners, H); // transform to get the scene_corner array
            perspectiveTransform( obj, scene_ideal, H); // transform to get the ideal scene where the points are expected to be

            // added - check to see the size of the found image by measuring the lengths of the perimeter of the square

            float curr_distance = 0.0;
            float min_distance = std::numeric_limits<float>::infinity();
            float max_distance = 0.0;

            float max_scene_y = 0.0;
            float max_scene_x = 0.0;
            float min_scene_y = std::numeric_limits<float>::infinity();
            float min_scene_x = std::numeric_limits<float>::infinity();

            for (int i = 0; i < 4; i++){

                // chech the side lengths of the found image

                if (i < 3){
                    curr_distance = eucDist(scene_corners[i].x, scene_corners[i].y, scene_corners[i+1].x, scene_corners[i+1].y);
                }
                else {
                    curr_distance = eucDist(scene_corners[i].x, scene_corners[i].y, scene_corners[0].x, scene_corners[0].y);
                }
                
                if (curr_distance < min_distance){
                    min_distance = curr_distance;
                }
                if (curr_distance > max_distance){
                    max_distance = curr_distance;
                }

                // check the max x and y coordinates of the image

                if (scene_corners[i].y > max_scene_y){
                    max_scene_y = scene_corners[i].y;
                }
                if (scene_corners[i].y < min_scene_y){
                    min_scene_y = scene_corners[i].y;
                }
                if (scene_corners[i].x > max_scene_x){
                    max_scene_x = scene_corners[i].x;
                }
                if (scene_corners[i].x < min_scene_x){
                    min_scene_x = scene_corners[i].x;
                }

            }

            // if the points are all bundles on top of each other then remove them
            // and the matching points are very far from the scene times the homogrpahy then remove those too

            // if matches are outside the max x and y coordiantes of the found image then remove them

            std::vector<Point2f> scene_better;

            for (int i = 1; i < scene.size(); i++)
            {
                if (eucDist(scene[i].x, scene[i].y, scene[i-1].x, scene[i-1].y) > minDistTHRESH && eucDist(scene[i].x, scene[i].y, scene_ideal[i].x, scene_ideal[i].y) < HTHRESH)
                {        
                    if (scene[i].y < max_scene_y && scene[i].y > min_scene_y && scene[i].x < max_scene_x && scene[i].x > min_scene_x){
                        scene_better.push_back(scene[i]);
                    }
                }
                
            }

            int NumMatches = scene_better.size(); // Value to Output !!!!!!!!!!!!!!!!!!!!!!!!!!

            // if plotting the output, but this does not include all the filters put onto the output

            if (visual)
            {

            //-- Draw the lines between matches
                Mat img_matches;
                drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                            Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


                //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
                    scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
                line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
                    scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
                    scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
                    scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                //-- Show detected matches
                imshow("Good Matches & Object detection", img_matches );
                waitKey(10);
            }

            // added - scale down the output value if the image size seems off

            if (min_distance < 50){
                NumMatches = (-(2/(min_distance + 2)) + 1) * NumMatches;
                
            }
            else if (max_distance > 1000){
                NumMatches = ((-1/2000)*max_distance + 1.5) * NumMatches;
            }

            // scale down if the image is greatly skewed

            if (min_distance > 0 && NumMatches > 0)
            {
                if (max_distance/min_distance > SKEWFACTOR){
                    NumMatches = SKEWFACTOR/NumMatches;
                }

            }
            else {
                NumMatches = 0;
            }

            return NumMatches;
        }
    }

    return 0;
}



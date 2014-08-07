/**
 * @brief SURF detector + descriptor + FLANN Matcher + FindHomography
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace std;
using namespace Eigen;
using namespace cv;

typedef std::vector<cv::Point2f > Keypoints;
typedef std::pair< Keypoints,Keypoints > Correspondances;

bool read_file(std::ifstream& is, Keypoints& k)
{
    std::string line;
    Point2f point;
    while(std::getline(is, line))
    {
        std::stringstream ss(line);
        ss >> point.x >> point.y;
        k.push_back(point);
    }
//    cout << k.size() << endl;
    is.close();
    return true;
}


/**
 * @brief Main function
 */
int main( int argc, char** argv )
{
    if( argc != 3 ) {
        cout << " Usage: ./test_homographyRansac <features1> <features2>" << endl;
        return -1;
    }
//    cout << argv[1] << ' ' << argv[2] << endl;
    ifstream feas_odom;
    ifstream feas_map;
    feas_odom.open(argv[1]);
    feas_map.open(argv[2]);

{
    //  Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
    //  Mat img_scene = imread( argv[2], IMREAD_GRAYSCALE );

    //  if( !img_object.data || !img_scene.data )
    //  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

    //  //-- Step 1: Detect the keypoints using SURF Detector
    //  int minHessian = 400;

    //  SurfFeatureDetector detector( minHessian );

    //  std::vector<KeyPoint> keypoints_object, keypoints_scene;

    //  detector.detect( img_object, keypoints_object );
    //  detector.detect( img_scene, keypoints_scene );

    //  //-- Step 2: Calculate descriptors (feature vectors)
    //  SurfDescriptorExtractor extractor;

    //  Mat descriptors_object, descriptors_scene;

    //  extractor.compute( img_object, keypoints_object, descriptors_object );
    //  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //  //-- Step 3: Matching descriptor vectors using FLANN matcher
    //  FlannBasedMatcher matcher;
    //  std::vector< DMatch > matches;
    //  matcher.match( descriptors_object, descriptors_scene, matches );

    //  double max_dist = 0; double min_dist = 100;

    //  //-- Quick calculation of max and min distances between keypoints
    //  for( int i = 0; i < descriptors_object.rows; i++ )
    //  { double dist = matches[i].distance;
    //    if( dist < min_dist ) min_dist = dist;
    //    if( dist > max_dist ) max_dist = dist;
    //  }

    //  printf("-- Max dist : %f \n", max_dist );
    //  printf("-- Min dist : %f \n", min_dist );

    //  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    //  std::vector< DMatch > good_matches;

    //  for( int i = 0; i < descriptors_object.rows; i++ )
    //  { if( matches[i].distance < 3*min_dist )
    //    { good_matches.push_back( matches[i]); }
    //  }

    //  Mat img_matches;
    //  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
    //               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
    //               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
}

    // creatign the good matches
    Keypoints keypoints_odom;
    Keypoints keypoints_map;

    Correspondances good_matches;
    //reading from the file txt and creating the correspondances vector
    read_file(feas_odom,keypoints_odom);
    read_file(feas_map,keypoints_map);

    //for now manual good matches---> automatic find good correspondances needed..
    good_matches = make_pair(keypoints_odom, keypoints_map);

    Keypoints k_odom, k_map;
    for(size_t i = 0; i < good_matches.first.size(); i++) {

        //get the keypoints from teh good matches
        k_odom = good_matches.first;
        k_map = good_matches.second;

        // debug
        cout << k_odom[i] << " <--> " << k_map[i] << endl;
    }
        Mat H = findHomography(k_odom, k_map, RANSAC);

        // debug
        cout << "Homography H = "<< endl << " "  << H << endl << endl;

        Keypoints k_odomToMap;
        perspectiveTransform( k_odom, k_odomToMap, H);
        ofstream os("odomtoMap.dat");
        for (size_t i = 0; i < k_odomToMap.size(); i++)
        {
            cout << "Keypoints odom perspective transformed on the map\n" << k_odomToMap[i] << endl;
            Vector2f odomToMap(Eigen::Vector2f(k_odomToMap[i].x, k_odomToMap[i].y));
            os << odomToMap.transpose() << endl;
        }

{
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<Point2f> obj_corners(4);
//    obj_corners[0] = Point(0,0); obj_corners[1] = Point( img_object.cols, 0 );
//    obj_corners[2] = Point( img_object.cols, img_object.rows ); obj_corners[3] = Point( 0, img_object.rows );
//    std::vector<Point2f> scene_corners(4);

//    perspectiveTransform( obj_corners, scene_corners, H);


//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    Point2f offset( (float)img_object.cols, 0);
//    line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4 );
//    line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );

//    //-- Show detected matches
//    imshow( "Good Matches & Object detection", img_matches );
}
    waitKey(0);

    return 0;
}

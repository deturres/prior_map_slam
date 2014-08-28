/**
 * @brief SURF detector + descriptor + FLANN Matcher + FindHomography
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


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

//compute the Homography and find the perspective transformed points with it
void findHomography(Correspondances& good_matches, Mat H, Keypoints k_odomToMap) {

    Keypoints k_odom, k_map;
    for(size_t i = 0; i < good_matches.first.size(); i++) {

        //get the keypoints from teh good matches
        k_odom = good_matches.first;
        k_map = good_matches.second;

        // debug
        cout << k_odom[i] << " <--> " << k_map[i] << endl;
    }
    H = findHomography(k_odom, k_map, RANSAC);

    // debug
    cout << "Homography H = "<< endl << " "  << H << endl << endl;
    perspectiveTransform( k_odom, k_odomToMap, H);

    // (visual)debug
    ofstream os("odomtoMap.dat");
    for (size_t i = 0; i < k_odomToMap.size(); i++)
    {
        cout << "Keypoints odom perspective transformed on the map\n" << k_odomToMap[i] << endl;
        Vector2f odomToMap(Eigen::Vector2f(k_odomToMap[i].x, k_odomToMap[i].y));
        os << odomToMap.transpose() << endl;
    }

    //extracting the Scalar value from the Homgraphy so that map = S * odom
    //normalization
    int a = sgn(H.at<double>(2,2));
    float b = sqrt(pow(H.at<double>(2,0),2) + pow(H.at<double>(2,1), 2) + pow(H.at<double>(2,2),2));
    float eta = a/b;
    H = eta*H;

    //svd decomposition
    SVD svd(H);
    Mat U = svd.u;
    Mat Vt =svd.vt.t();
    //extracting scale
    Mat D = Mat::diag(svd.w);
    //extracting theta
    Mat R = svd.u*svd.vt; //vt transpose???
    double theta = atan2(R.at<double>(1,0),R.at<double>(0,0));

    cout << "eta = " << endl << " "  << eta << endl << endl;
    cout << "eta*H = " << endl << " "  << H << endl << endl;
    cout << "D = " << endl << " "  << D << endl << endl;
    cout << "R = " << endl << " "  << R <<  ", with theta: "<< theta << endl << endl;


//    JacobiSVD<Matrix2d> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    if (svd.singularValues()(0)<.5) {
//        cout << "problems with homography singular values" << endl;
//        Matrix2d R = svd.matrixU()*svd.matrixV().transpose();
//        Matrix3d D = svd.singularValues();
//    }



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

    // creating the good matches
    Keypoints keypoints_odom;
    Keypoints keypoints_map;

    Correspondances good_matches;
    //reading from the file txt and creating the correspondances vector
    read_file(feas_odom,keypoints_odom);
    read_file(feas_map,keypoints_map);

    //for now manual good matches---> automatic find good correspondances needed..
    good_matches = make_pair(keypoints_odom, keypoints_map);
    Mat H;
    Keypoints k_odomToMap;
    findHomography(good_matches, H, k_odomToMap);

    return 0;
}

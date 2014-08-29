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
void findHomographyM(Correspondances& good_matches, Mat& H, Keypoints& k_mapToOdom) {

    Keypoints k_map, k_odom;
    for(size_t i = 0; i < good_matches.first.size(); i++) {

        //get the keypoints from teh good matches
        k_map = good_matches.first;
        k_odom = good_matches.second;

        // debug
        cout << k_map[i] << " <--> " << k_odom[i] << endl;
    }
    H = findHomography(k_map, k_odom, RANSAC);

    // debug
    cout << "Homography H = "<< endl << " "  << H << endl << endl;
    perspectiveTransform(k_map, k_mapToOdom, H);

    // (visual)debug
    ofstream os("mapToOdom.dat");
    for (size_t i = 0; i < k_mapToOdom.size(); i++)
    {
        cout << "Keypoints map perspective transformed on the odom\n" << k_mapToOdom[i] << endl;
        Vector2f mapToOdom(Eigen::Vector2f(k_mapToOdom[i].x, k_mapToOdom[i].y));
        os << mapToOdom.transpose() << endl;
    }

//    /// decomposeHomography() is not working !!!!!!!!!!!!!!!
//    //svd(H) to find Scale, Rotation angle and Translation
//    //normalization
//    int a = sgn(H.at<double>(2,2));
//    double b = sqrt(pow(H.at<double>(2,0),2) + pow(H.at<double>(2,1), 2) + pow(H.at<double>(2,2),2));
//    double eta = a/b;
//    H = eta*H;
////    cout << "eta = " << endl << " "  << eta << endl << endl;
////    cout << "eta*H = " << endl << " "  << H << endl << endl;

//    //extracting P pure projection matrix
//    Mat P = H.clone();
//    Mat I = Mat::eye(2,3,P.type());
//    I.copyTo(P.colRange(0,3).rowRange(0,2));
////    cout << " P " << endl << " "  << P << endl << endl;

//    // H affine = H*P^-1
//    Mat Ha =  H*P.inv();
//    cout << " Ha " << endl << " "  << Ha << endl << endl;

//    //svd decomposition
//    SVD svd(Ha);
//    //extracting scale
//    Mat S = Mat::diag(svd.w);
//    //extracting theta
//    Mat R = svd.u*svd.vt.t();
//    double theta = atan2(R.at<double>(1,0),R.at<double>(0,0));
//    //extracting traslation
//    Vector3d t = Eigen::Vector3d(Ha.at<double>(2,0),Ha.at<double>(2,1),1);

//    cout << "S = " << endl << " "  << S << endl << endl;
//    cout << "R = " << endl << " "  << R <<  ", with theta: "<< theta << endl << endl;
//    cout << "t = " << endl << " "  << t << endl << endl;

}

void decomposeHomography(Mat& H, Mat& S, Mat& R, Vector3d& t){

    //normalization
    int a = sgn(H.at<double>(2,2));
    double b = sqrt(pow(H.at<double>(2,0),2) + pow(H.at<double>(2,1), 2) + pow(H.at<double>(2,2),2)); //norm !!!!
    double eta = a/b;
    H = eta*H;
//    cout << "eta = " << endl << " "  << eta << endl << endl;
//    cout << "eta*H = " << endl << " "  << H << endl << endl;

    //extracting P pure projection matrix
    Mat P = H.clone();
    Mat I = Mat::eye(2,3,P.type());
    I.copyTo(P.colRange(0,3).rowRange(0,2));
//    cout << " P " << endl << " "  << P << endl << endl;

    // H affine = H*P^-1
    Mat Ha =  H*P.inv();
    cout << " Ha " << endl << " "  << Ha << endl << endl;

    //extracting traslation
    double fuck = Ha.at<double>(2,0);
    cout << fuck << endl;
    t = Eigen::Vector3d(Ha.at<double>(0,2),Ha.at<double>(1,2),1);
    //svd decomposition to extract R
    SVD svd(Ha);
    //extracting scale
    S = Mat::diag(svd.w);
    //extracting theta
    R = svd.u*svd.vt.t();
    double theta = atan2(R.at<double>(1,0),R.at<double>(0,0));

//    cout << "S = " << endl << " "  << S << endl << endl;
//    cout << "R = " << endl << " "  << R <<  ", with theta: "<< theta << endl << endl;
//    cout << "t = " << endl << " "  << t << endl << endl;

//    JacobiSVD<Matrix2d> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    if (svd.singularValues()(0)<.5) {
//        cout << "problems with homography singular values" << endl;
//        Matrix2d R = svd.matrixU()*svd.matrixV().transpose();
//        Matrix3d D = svd.singularValues();
//    }
}

void constraintDefinition(Correspondances& good_matches, Mat& S, Mat& R, Vector3d& t) {
    //creating constraints between features points
    // f(0) = (0,0);
    // f(x) = S * R * [u,v]_x - [u,v]_0 :: per ogni X



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
    ifstream feas_map;
    ifstream feas_odom;
    feas_map.open(argv[1]);
    feas_odom.open(argv[2]);

    // creating the good matches
    Keypoints keypoints_map;
    Keypoints keypoints_odom;

    Correspondances good_matches;
    //reading from the file txt and creating the correspondances vector
    read_file(feas_map,keypoints_map);
    read_file(feas_odom,keypoints_odom);

    //for now manual good matches---> automatic find good correspondances needed..
    good_matches = make_pair(keypoints_map, keypoints_odom);
    Mat H;
    Keypoints k_mapToOdom;
    //find homography
    findHomographyM(good_matches, H, k_mapToOdom);
    cout << "Homography H = "<< endl << " "  << H << endl << endl;

    //decompose homography
    Mat S,R;
    Vector3d t;
    decomposeHomography(H, S, R, t);
    cout << "S = " << endl << " "  << S << endl << endl;
    cout << "R = " << endl << " "  << R << endl << endl;
    cout << "t = " << endl << " "  << t << endl << endl;

    constraintDefinition(good_matches, S, R, t);

    return 0;
}

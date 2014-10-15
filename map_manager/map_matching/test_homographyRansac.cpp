/**
 * @brief SURF detector + descriptor + FLANN Matcher + FindHomography
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "utility.h"

using namespace std;
using namespace Eigen;
using namespace cv;

//typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef std::pair< int,Vector2d > NewPoseSeq;
typedef std::vector<NewPoseSeq, Eigen::aligned_allocator<Vector2d> > NewPoses;
//typedef std::vector<Eigen::Vector2d > NewPoses;

//typedef std::vector<cv::Point2f > Keypoints;
typedef std::pair< int,cv::Point2f > KeypointsSeq;
typedef std::vector<KeypointsSeq > Keypoints;
typedef std::pair< Keypoints,Keypoints > Correspondances;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool read_file_m(std::ifstream& is, Keypoints& k)
{
    //todo aggiungi pair con seq = -1 (check the struct)
    std::string line;
    Point2f point;
    while(std::getline(is, line))
    {
        std::stringstream ss(line);
        ss >> point.x >> point.y;
        //reading a map (planimetry, no sequential number available)
        k.push_back(make_pair(-1,point));
    }
//    cout << k.size() << endl;
    is.close();
    return true;
}

bool read_file_o(std::ifstream& is, Keypoints& k, Vector<Vector4f>& r)
{
    //todo aggiungi pair con seq (check the struct)

    std::string line;
    Point2f point;
    Vector4f rest;
    int seq;
    while(std::getline(is, line))
    {
        std::stringstream ss(line);       
        ss >> seq;
        ss >> point.x >> point.y >> rest[0] >> rest[1] >> rest[2] >> rest[3];
        k.push_back(make_pair(seq,point));
        r.push_back(rest);

    }
//    cout << k.size() << endl;
    is.close();
    return true;
}

//compute the Homography and find the perspective transformed points with it
void findHomographyM(Correspondances& good_matches, Mat& H) {

    Keypoints k_map, k_odom;
    //get the keypoints from teh good matches
    k_map = good_matches.first;
    k_odom = good_matches.second;
    std::vector<cv::Point2f> k_mapV, k_odomV;
        for(size_t i = 0; i < k_map.size(); i++) {
            //they have the same size
            k_mapV.push_back(k_map[i].second);
            k_odomV.push_back(k_odom[i].second);
        }
    //debug
    for(size_t i = 0; i < good_matches.first.size(); i++) {
//        KeypointsSeq a = k_map[i];
        cout << k_map[i].first << k_mapV[i] << " <--> " << k_odomV[i] << endl;
    }
    H = findHomography(k_mapV, k_odomV, RANSAC);
    // debug
    cout << "Homography H = "<< endl << " "  << H << endl << endl;

//    perspectiveTransform(k_map, k_mapToOdom, H);
//    // (visual)debug
//    ofstream os("mapToOdom.dat");
//    for (size_t i = 0; i < k_mapToOdom.size(); i++)
//    {
//        cout << "Keypoints map perspective transformed on the odom\n" << k_mapToOdom[i] << endl;
//        Vector2f mapToOdom(Eigen::Vector2f(k_mapToOdom[i].x, k_mapToOdom[i].y));
//        os << mapToOdom.transpose() << endl;
//    }
}


//svd(H) to find Scale, Rotation angle and Translation
void decomposeHomography(Mat& H, Matrix2d& S, Matrix2d& R, Vector2d& t){

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
    t = Eigen::Vector2d(Ha.at<double>(0,2),Ha.at<double>(1,2));

    // Mat::svd decomposition to extract R :: change everything in 3d
//    SVD svd(Ha, 1);
//    //extracting scale
//    Mat D = Mat::diag(svd.w);
//    cv2eigen(D,S);
//    //extracting theta
//    Mat UVt = svd.u*svd.vt.t();
//    cv2eigen(UVt,R);

    Matrix3d Haffine = Matrix3d::Zero();
    cv2eigen(Ha, Haffine);
    JacobiSVD<Matrix2d> svd(Haffine.block<2,2>(0,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues()(0)<.5) {
        cout << "problems with homography singular values" << endl;
        return;
    }
    R = svd.matrixU()*svd.matrixV().transpose();
    S <<
         svd.singularValues()(0), 0,
         0, svd.singularValues()(1);

    //debug
    cout << " Haffine " << endl << " "  << Haffine << endl << endl;
}


NewPoses newPoses(Correspondances& good_matches, Mat& H,  Keypoints& k_mapToOdom, Vector<Vector4f>& rest , Matrix2d& S, Matrix2d& R, Vector2d& t, std::ofstream& osNP) {
    //creating constraints between features points
    // f(0) = (0,0);
    // f(x_new) = S(R * [u,v]_x - [u,v]_0) per ogni matches

    Keypoints k_map, k_odom;
    //get the keypoints from the good matches
    k_map = good_matches.first;
    k_odom = good_matches.second;
    std::vector<cv::Point2f> k_mapV, k_odomV, k_mapToOdomV;
    for(size_t i = 0; i < k_map.size(); i++) {
        //they have the same size
        k_mapV.push_back(k_map[i].second);
        k_odomV.push_back(k_odom[i].second);
    }
    NewPoses newPoses;

    /// extracting R S and T to create the new poses: NOT WORKING!!
    //fixing the first point f(0) = (0,0);
    Vector2d p_odom_new  = Vector2d::Zero();
    Vector2d p_map = Vector2d::Zero();
    //per ogni matchesf(x_new) = S(R * [u,v]_x - [u,v]_0)

    for(size_t i = 0; i < good_matches.first.size(); i++) {

        if (i==0)
            p_odom_new = Eigen::Vector2d(k_odomV[i].x, k_odomV[i].y); // u,v;
        else
        {
            p_map = Eigen::Vector2d(k_mapV[i].x, k_mapV[i].y);
            p_odom_new = S*R*p_map + S*t; // u,v scaled, rotated and traslated according to H

        }
//        newPoses.push_back(p_odom_new);
    }

    /// BETTER CONSIDER THE COMPLETE HOMOGRAPY (P <PROJECTION> INCLUDED)
    perspectiveTransform(k_mapV, k_mapToOdomV, H);
//    ofstream os("mapToOdom.dat");
    cout << "Keypoints map perspective transformed on the odom\n" << endl;
    for (size_t i = 0; i < k_mapToOdomV.size(); i++)
    {
        int seq = k_odom[i].first;
        k_mapToOdom.push_back(make_pair(seq,k_mapToOdomV[i]));
//        cout << "Keypoints map perspective transformed on the odom\n" << k_mapToOdom[i] << endl;
        Vector2d mapToOdom(Eigen::Vector2d(k_mapToOdomV[i].x, k_mapToOdomV[i].y));
        newPoses.push_back(make_pair(seq,mapToOdom));
        osNP << seq << " " << mapToOdom.transpose() << " " << rest[i].transpose() << endl;
    }
    //debug
    for (size_t i = 0; i < newPoses.size(); i++)
        cout << k_odomV[i] << " <--> " << newPoses[i].second.transpose() << endl;

    return newPoses;
}


/**
 * @brief Main function
 */
int main( int argc, char** argv )
{
    if( argc != 4 ) {
        cout << " Usage: ./test_homographyRansac <features_map> <features_odom> <new Poses Filename>" << endl;
        return -1;
    }
    ifstream feas_map;
    ifstream feas_odom;
    ofstream newPoses_odom/*("newPoses.txt")*/;

    feas_map.open(argv[1]);
    feas_odom.open(argv[2]);
    newPoses_odom.open(argv[3]);

    // creating the good matches
    Keypoints keypoints_map;
    Keypoints keypoints_odom;
    Vector<Vector4f> keypoints_rest;

    Correspondances good_matches;
    //reading from the file txt and creating the correspondances vector
    read_file_m(feas_map, keypoints_map);
    read_file_o(feas_odom, keypoints_odom, keypoints_rest);

    //for now manual good matches---> automatic find good correspondances needed..
    good_matches = make_pair(keypoints_map, keypoints_odom);
    Mat H;

    //find homography
    findHomographyM(good_matches, H);
    cout << "Homography H = "<< endl << " "  << H << endl << endl;

    //decompose homography
    Matrix2d S,R;
    Vector2d t;
    decomposeHomography(H, S, R, t);
    double thetaR = atan2(R(1,0),R(0,0));
    cout << "S = " << endl << " "  << S << endl << endl;
    cout << "R = " << endl << " "  << R << " with thetaR " << thetaR << endl << endl;
    cout << "t = " << endl << " "  << t << endl << endl;

    Keypoints k_mapToOdom;
    newPoses(good_matches, H, k_mapToOdom, keypoints_rest, S, R, t, newPoses_odom);

    return 0;
}

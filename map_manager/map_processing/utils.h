#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace cv;

void computeHoughTransform(Mat &src, Mat &cdst) {

    Mat dst;
    //applies Canny edge detector and produces the edge map.
    Canny(src, dst, 50, 200, 3);
    imshow("edges", dst);
    cvtColor(dst, cdst, CV_GRAY2BGR);

     #if 0
      vector<Vec2f> lines;
      //compute the standard Hough Transform
      HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

      for( size_t i = 0; i < lines.size(); i++ )
      {
         float rho = lines[i][0], theta = lines[i][1];
         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
         line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
      }
     #else
      vector<Vec4i> lines;
      //compute the probabilistic Hough Transform: rho = 1pixel, theta = 1 degree
      HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
      for( size_t i = 0; i < lines.size(); i++ )
      {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
      }
     #endif
     imshow("source", src);
     imshow("detected lines", cdst);
}

void computeSkeleton(Mat &src, Mat &dst) {
    // transform teh image to a binary image using thresholding:
    cv::threshold(src, src, 127, 255, cv::THRESH_BINARY);
    //    skel = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat skel(src.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(src.size(), CV_8UC1);

    // here we use a 3x3 cross-shaped structure element (i.e. we use 4-connexity)
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    //main loop
    bool done = false;
    while(!done)
    {
        cv::morphologyEx(src, temp, cv::MORPH_OPEN, element);
        cv::bitwise_not(temp, temp);
        cv::bitwise_and(src, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        cv::erode(src, src, element);

        double max;
        cv::minMaxLoc(src, 0, &max);
        done = (max == 0);
    }
    cv::imshow("Skeleton", skel);
    dst = skel;
}


//void help()
//{
// cout << "\nThis program demonstrates a skeletal transform.\n"
//         "Usage:\n"
//         "./skeletal <image_name>, Default is pic1.jpg\n" << endl;
//}

//void compSkel2(Mat& src){

// if(src.empty())
// {
//     help();
//     cout << "can not open " << filename << endl;
//     return -1;
// }

// // show original source image and wait for input to next step
// imshow("source", src);
// waitKey();


// cvtColor(src, gray, CV_BGR2GRAY);

// // show graymap of source image and wait for input to next step
// imshow("graymap", gray);
// waitKey();


// // Use 70 negative for Moose, 150 positive for hand
// //
// // To improve, compute a histogram here and set threshold to first peak
// //
// // For now, histogram analysis was done with GIMP
// //
// threshold(gray, binary, 70, 255, CV_THRESH_BINARY);
// binary = 255 - binary;

// // show bitmap of source image and wait for input to next step
// imshow("binary", binary);
// waitKey();

// // To remove median filter, just replace blurr value with 1
// medianBlur(binary, mfblur, 1);

// // show median blur filter of source image and wait for input to next step
// //imshow("mfblur", mfblur);
// //waitKey();

// // This section of code was adapted from the following post, which was
// // based in turn on the Wikipedia description of a morphological skeleton
// //
// // http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
// //
// Mat skel(mfblur.size(), CV_8UC1, Scalar(0));
// Mat temp;
// Mat eroded;
// Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
// bool done;
// int iterations=0;

// do
// {
//   erode(mfblur, eroded, element);
//   dilate(eroded, temp, element);
//   subtract(mfblur, temp, temp);
//   bitwise_or(skel, temp, skel);
//   eroded.copyTo(mfblur);

//   done = (countNonZero(mfblur) == 0);
//   iterations++;

// } while (!done && (iterations < 100));

// cout << "iterations=" << iterations << endl;

// imshow("skeleton", skel);
// waitKey();

// return 0;
//}




void printType(cv::Mat &mat) {
         if(mat.depth() == CV_8U)  std::printf("unsigned char(%d)", mat.channels());
    else if(mat.depth() == CV_8S)  std::printf("signed char(%d)", mat.channels());
    else if(mat.depth() == CV_16U) std::printf("unsigned short(%d)", mat.channels());
    else if(mat.depth() == CV_16S) std::printf("signed short(%d)", mat.channels());
    else if(mat.depth() == CV_32S) std::printf("signed int(%d)", mat.channels());
    else if(mat.depth() == CV_32F) std::printf("float(%d)", mat.channels());
    else if(mat.depth() == CV_64F) std::printf("double(%d)", mat.channels());
    else                           std::printf("unknown(%d)", mat.channels());
}

void printInfo(const char *prefix, Mat &mat) {
    printf("%s: ", prefix);
    printf("dim(%d, %d)", mat.rows, mat.cols);
    printType(mat);
    printf("\n");
}

void printInfo(Mat &mat) {
    printf("dim(%d, %d)", mat.rows, mat.cols);
    printType(mat);
    printf("\n");
}

void printValue(Mat &mat) {

    Mat::MSize s = mat.size;
    int w = s().width/5;
    int h = s().height/5;

    for(int x=0; x<w;x++)
      {
         cout<<endl;
         for(int y=0; y<h;y++)
             cout<<std::setw(10)<<mat.at<float>(x, y);
      }
}

#include "utils.h"

#define DEBUG_MODE 0

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

    if (argc != 5) {
      fprintf(stderr, "usage: %s input(pbm) output_dt(pgm)\n output_ht(pgm)\n output_sk(pgm)\n", argv[0]);
      return 1;
    }
    char *input_name = argv[1];
    char *output_name_dt = argv[2];
    char *output_name_f = argv[3];
    char *output_name_sk = argv[4];

    Mat before = imread(input_name, 0); //CV_LOAD_IMAGE_COLOR
    Mat dist, htdst, skdst;
    // computes the distance transform map using the simple euclidean distance
    distanceTransform(before, dist, CV_DIST_L2, 5);

#if DEBUG_MODE
    imshow("before the transform", before);
    imshow("non-normalized after the transform", dist);
#endif

    // Normalization: matrix decomposition types NORM_MINMAX
    normalize(dist, dist, 0.0, 1.0, NORM_MINMAX);
    imshow("normalized_distance_transform", dist);

    //writing the image
    dist.convertTo(dist,CV_8UC1,255);
#if DEBUG_MODE
    printInfo(dist);
//    printValue(dist);
#endif
    std::vector<int> qualityType;
    qualityType.push_back(CV_IMWRITE_PXM_BINARY);
    qualityType.push_back(1);
    imwrite(output_name_dt, dist, qualityType);

    // compute the Hough Probabilistic Transform for line detection on the original image
    computeHoughTransform(before, htdst);
    imwrite(output_name_f, htdst, qualityType);


    //compute skeletonization
    computeSkeleton(before, skdst);
    imwrite(output_name_sk, skdst, qualityType);

    waitKey();
    return 0;
}

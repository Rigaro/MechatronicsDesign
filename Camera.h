#ifndef CAMERA_H
#define CAMERA_H

#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat ImageProcessing(Mat, int, int);
Mat GetBallPosition(int*, int*, int*, vector<Vec3f>, Mat);

#endif // CAMERA_H

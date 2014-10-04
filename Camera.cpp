/*
 * @author Ricardo Garcia Rosas
 *
 * @section DESCRIPTION
 *
 * Couple methods to organize the image processing implemented with OpenCV.
 *
 */

#include "Camera.h"

//Processes an image to be used by the HoughCircles OpenCV function.
//@param source image.
//@param gray scale threshold.
//@param morph element size.
//@return processed image.
Mat ImageProcessing(Mat source, int thres, int morphSize)
{
    //TODO: Go over http://stackoverflow.com/questions/9860667/writing-robust-color-and-size-invariant-circle-detection-with-opencv-based-on
    Mat grayScale, thresholded;
    //Convert frame to Gray
    cvtColor(source, grayScale, CV_BGR2GRAY);

    // Blur out noise
    //GaussianBlur(grayScale, grayScale, Size(9, 9), 2, 2);

    //Threshold
    threshold(grayScale, thresholded, thres, 255, THRESH_BINARY);

    //Morph opening (remove small objects from foreground)
    erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));
    dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));
    //Morph closing (fill small holes from foreground)
    dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));
    erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));

    return thresholded;
}

Mat ImageProcessing(Mat source, int lowH, int highH, int lowS, int highS, int lowV, int highV, int morphSize)
{
    //TODO: Go over http://stackoverflow.com/questions/9860667/writing-robust-color-and-size-invariant-circle-detection-with-opencv-based-on
    Mat imgHSV, thresholded;

    // Blur out noise
    //GaussianBlur(source, source, Size(9, 9), 2, 2);

    //Convert from BGR to HSV
    cvtColor(source, imgHSV, COLOR_BGR2HSV);

    //Threshold the image
    inRange(imgHSV, Scalar(lowH,lowS,lowV), Scalar(highH,highS,highV), thresholded);


    //Morph opening (remove small objects from foreground)
    erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));
    dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));
    //Morph closing (fill small holes from foreground)
    dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));
    erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(morphSize,morphSize)));

    for(int rows = 0; rows < thresholded.rows; rows++) {
        for(int cols = 0; cols <= 18; cols++) {
            Vec3b point = thresholded.at<Vec3b>(Point(cols, rows));
            point.val[0] = 255;
            point.val[1] = 255;
            point.val[2] = 255;
            thresholded.at<Vec3b>(Point(cols, rows)) = point;
        }
    }

    for(int rows = 0; rows < thresholded.rows; rows++) {
        for(int cols = 612; cols <= thresholded.cols; cols++) {
            Vec3b point = thresholded.at<Vec3b>(Point(cols, rows));
            point.val[0] = 255;
            point.val[1] = 255;
            point.val[2] = 255;
            thresholded.at<Vec3b>(Point(cols, rows)) = point;
        }
    }
    return thresholded;
}

//Gets the ball position from the HoughCircles output and sets them to program variables.
//@param x position variable address pointer.
//@param y position variable address pointer.
//@param raidus variable address pointer.
//@param output vector from HoughCircles.
//@param source image to draw UI circles on.
//@return image with circles on top.
Mat GetBallPosition(int* xPosBall, int* yPosBall, int* radius, vector<Vec3f> circles, Mat source)
{
    if(circles.size() > 0) {
        *xPosBall = cvRound(circles[0][0]);
        *yPosBall = cvRound(circles[0][1]);
        *radius = cvRound(circles[0][2]);
        Point center(*xPosBall, *yPosBall);

        //Print ball position on frame
        circle( source, center, 3, Scalar(0,255,0), -1, 8, 0);
        circle( source, center, *radius, Scalar(0,0,255), 3, 8, 0);
    }
    return source;
}

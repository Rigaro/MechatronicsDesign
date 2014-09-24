#include "SendSerial.h"
#include "Camera.h"
#include "controller.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#define INFINITE_LOOP true

using namespace cv;
using namespace std;

int SerialTest();

int main(int argc, char** argv)
{
    int thresh = 62;
    int upperThres = 10;
    int centerThres = 10;
    int xPosBall, yPosBall, radius, xAngle, yAngle;

    Mat source, processed;
    vector<Vec3f> circles;
    //Get video
    VideoCapture cap(2);

    Controller xControl(0,15,1);
    //Controller yControl(0,15,1);
    xControl.SetDesiredPos(150);
    //yControl.SetDesiredPos(300);

    namedWindow("Original", CV_WINDOW_AUTOSIZE);

    //Trackbar for circle detection tunning.
    cvCreateTrackbar("Threshold","Original",&thresh,255);
    cvCreateTrackbar("Upper","Original",&upperThres,200);
    cvCreateTrackbar("Center","Original",&centerThres,200);

    if(!cap.isOpened())
    {
        cout << "can not open video" << endl;
        return -1;
    }

    while(INFINITE_LOOP)
    {
        bool bSuccess = cap.read(source);

        if(!bSuccess)
        {
            cout << "Can't read frame from video" << endl;
            break;
        }

        processed = ImageProcessing(source, thresh, 9);

        //Get circles from processed image.
        HoughCircles(processed, circles, CV_HOUGH_GRADIENT, 1, processed.rows/8, upperThres, centerThres, 15, 25);

        source = GetBallPosition(&xPosBall, &yPosBall, &radius, circles, source);

        xAngle = xControl.PositionControl(xPosBall);

        cout << "x ang: " << xAngle << endl;
        cout << "x pos: " << xPosBall << endl;
        cout << "des pos: " << xControl.GetDesiredPos() << endl;
        cout << "cur pos: " << xControl.GetCurrentPos() << endl;
        cout << "error: " << xControl.GetErrorNew() << endl;
        //cout << "y: " << yPosBall << endl;
        //cout << "r: " << radius << endl;
        imshow("Original", source);
        imshow("Thresh", processed);

        //Stop process when esc is pressed.
        if(waitKey(30) == 27)
        {
            break;
        }
    }
}


//Test program for serial communication with arduino.
//Sends trackbar value for x and y angle.
int SerialTest()
{
    int xAngle = 0;
    int yAngle = 0;

    Mat src;

    VideoCapture cap(2);

    if(!cap.isOpened())
    {
        cout << "can not open video" << endl;
        return -1;
    }

    namedWindow("Test", CV_WINDOW_AUTOSIZE);

    //Trackbar
    cvCreateTrackbar("X Angle","Test",&xAngle,15);
    cvCreateTrackbar("Y Angle","Test",&yAngle,15);

    while(true)
    {
        bool bSuccess = cap.read(src);

        if(!bSuccess)
        {
            cout << "Can't read frame from video" << endl;
            break;
        }

        //Send x data
        SendSerial(xAngle, 'x', PORT_1);
        //Send y data
        SendSerial(yAngle, 'y', PORT_1);

        imshow("Test", src);

        if(waitKey(30) == 27)
        {
            break;
        }
    }
    return 0;
}

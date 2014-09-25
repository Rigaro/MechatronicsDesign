#include "SendSerial.h"
#include "Camera.h"
#include "controller.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#define INFINITE_LOOP true
#define BOARD_0_XANG 7
#define BOARD_0_YANG 7

using namespace cv;
using namespace std;

int SerialTest();

int main(int argc, char** argv)
{
    SerialTest();
}

int MainProgram(){
    bool ballOnBoard = false;
    int thresh = 62;
    int upperThres = 10;
    int centerThres = 10;
    int xPosBall, yPosBall, radius;
    int xAngle = BOARD_0_XANG;
    int yAngle = BOARD_0_YANG;

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

        ballOnBoard = circles.size() != 0;

        if(ballOnBoard) {
            source = GetBallPosition(&xPosBall, &yPosBall, &radius, circles, source);
            xAngle = xControl.PositionControl(xPosBall);
        }
        else {
            xAngle = BOARD_0_XANG;
            yAngle = BOARD_0_YANG;
        }

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
    int xAngle = 65;
    int yAngle = 105;

    Mat src;

    VideoCapture cap(2);

    if(!cap.isOpened())
    {
        cout << "can not open video" << endl;
        return -1;
    }

    namedWindow("Test", CV_WINDOW_AUTOSIZE);

    //Trackbar
    cvCreateTrackbar("X Angle","Test",&xAngle,180);
    cvCreateTrackbar("Y Angle","Test",&yAngle,180);

    while(true)
    {
        bool bSuccess = cap.read(src);

        if(!bSuccess)
        {
            cout << "Can't read frame from video" << endl;
            break;
        }

        //Send x data
        SendSerial(xAngle, 'x', PORT_0);
        //Send y data
        SendSerial(yAngle, 'y', PORT_0);

        imshow("Test", src);

        if(waitKey(30) == 27)
        {
            break;
        }
    }
    return 0;
}

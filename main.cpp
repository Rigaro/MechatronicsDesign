#include "SendSerial.h"
#include "Camera.h"
#include "controller.h"
#include "controllerpid.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include <iostream>
#include <stdio.h>

#if defined(WIN32) || defined(WIN64)
#include <Windows.h>
#endif

#define INFINITE_LOOP true
#define BOARD_0_XANG 8
#define BOARD_0_YANG 8

#define CAM_INDEX 0

#define FPS_MAX 24

using namespace cv;
using namespace std;

int SerialTest();
int MainProgram();
double getAverageFPS();

int main(int argc, char** argv)
{
    MainProgram();

    system("pause");
}

double getAverageFPS()
{
    static double startTime = (double)getTickCount();
    static long frameCount = 0;

    double currTime = (double)getTickCount();
    frameCount++;

    double avgFps = frameCount / ((currTime - startTime) / getTickFrequency());

    if (avgFps > FPS_MAX)
        avgFps = FPS_MAX;

    return avgFps;
}

int MainProgram()
{
    bool ballOnBoard = false;
    int thresh = 105;
    int upperThres = 8;
    int centerThres = 8;
    int xPosBall = 0, yPosBall = 0, radius = 0;
    int xAngle = BOARD_0_XANG, prevXAngle = xAngle;
    int yAngle = BOARD_0_YANG, prevYAngle = yAngle;

    /* 
    To determine our sample frequency, we must determine the frame rate of the
    camera, as this is the frequency at which we receive updates.
    i.e. VideoCapture.read is a blocking method, so we will only process when
    we have a frame available. Therefore, the frame rate dictates our sample
    frequency.

    In ideal conditions, our FPS sits on 24.
    */
    double currTime = 0, beforeCapTime = 0, prevTime = 0;
    double frameTime = 0, frameDelta = 0;
    double averageFPS = 0;

    Mat source, processed;
    vector<Vec3f> circles;
    //Get video
    VideoCapture cap(CAM_INDEX);

    ControllerPID xControl(0, 16, 0.1, 0.1, 0.01);
    ControllerPID yControl(0, 16, 0.1, 0.1, 0.01);
    xControl.SetDesiredPos_px(150);
    yControl.SetDesiredPos_px(300);

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

    averageFPS = getAverageFPS();

    while (INFINITE_LOOP)
    {
        beforeCapTime = (double)getTickCount();

        bool bSuccess = cap.read(source);

        currTime = (double)getTickCount();

        // Get frame time and average FPS
        frameTime = (currTime - beforeCapTime) / getTickFrequency();
        averageFPS = getAverageFPS();
        frameDelta = (currTime - prevTime) / getTickFrequency();

        printf("Frame time: %0.4f, FPS: %0.3f, Average FPS: %0.4f, Frame delta: %0.2f\n", 
            frameTime, 1/frameTime, averageFPS, frameDelta);

        prevTime = currTime;

        if (!bSuccess)
        {
            cout << "Can't read frame from video" << endl;
            break;
        }

        processed = ImageProcessing(source, thresh, 9);

        //Get circles from processed image.
        HoughCircles(processed, circles, CV_HOUGH_GRADIENT, 1, processed.rows/8, 
                     upperThres, centerThres, 15, 25);

        ballOnBoard = circles.size() != 0;

        if(ballOnBoard) 
        {
            source = GetBallPosition(&xPosBall, &yPosBall, &radius, circles, 
                                     source);

            /* 
            We now have the position of the ball based on the pixels in the
            source stream. The origin (0,0) coordinate is the top left of the
            frame. We need to pass in the current ball position and the
            frameDelta (i.e. dt) for our calculations.
            */
            prevXAngle = xAngle;
            prevYAngle = yAngle;

            xAngle = xControl.PositionControl(xPosBall, frameDelta);
            yAngle = yControl.PositionControl(yPosBall, frameDelta);
        }

        

        //Send x data
        SendSerial(xAngle, 'x', PORT_0);
        //Send y data
        SendSerial(yAngle, 'y', PORT_0);

        printf("C X ang: %d, D X ang: %d, x pos: %d, x error: %d\n", xAngle,
            prevXAngle, xPosBall, xControl.GetCurrentError());

        printf("C Y ang: %d, D Y ang: %d, y pos: %d, y error: %d\n", yAngle,
            prevYAngle, yPosBall, yControl.GetCurrentError());

        imshow("Original", source);
        imshow("Thresh", processed);
        
        //Sleep(100);

        //Stop process when esc is pressed.
        if(waitKey(30) == 27)
        {
            break;
        }
    }

    return 0;
}

//Test program for serial communication with arduino.
//Sends trackbar value for x and y angle.
int SerialTest()
{
    int xAngle = 65;
    int yAngle = 98;

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

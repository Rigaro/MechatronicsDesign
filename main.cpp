#include "SendSerial.h"
#include "Camera.h"
#include "controller.h"
#include "controllerpid.h"
#include "tiltcontroller.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include <iostream>
#include <stdio.h>

#ifdef UNIX
#include <unistd.h>
#endif

#if defined(WIN32) || defined(WIN64)
#include <Windows.h>
#endif

#define INFINITE_LOOP true
#define BOARD_0_XANG 8
#define BOARD_0_YANG 8

#define CAM_INDEX 2

#define FPS_MAX 24

using namespace cv;
using namespace std;

int SerialTest();
int MainProgram();
double getAverageFPS();

int main(int argc, char** argv)
{
    MainProgram();
    //SerialTest();

    return 0;
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
    int upperThres = 10;
    int centerThres = 5;
    int xPosBall = 0, yPosBall = 0, radius = 0;
    int xAngle = BOARD_0_XANG, prevXAngle = xAngle;
    int yAngle = BOARD_0_YANG, prevYAngle = yAngle;
    int integralNum = 1, integralDen = 100, derivativeNum = 0, derivativeDen = 5, propNum = 30, propDen = 1;

    /* 
    To determine our sample frequency, we must determine the frame rate of the
    camera, as this is the frequency at which we receive updates.
    i.e. VideoCapture.read is a blocking method, so we will only process when
    we have a frame available. Therefore, the frame rate dictates our sample
    frequency.

    In ideal conditions, our FPS sits on 24.
    */
    double currTime = 0, beforeCapTime = 0, prevTime = (double)getTickCount();
    double frameTime = 0, frameDelta = 0;
    double averageFPS = 0;

    Mat source, processed;
    vector<Vec3f> circles;
    //Get video
    VideoCapture cap(CAM_INDEX);

    namedWindow("Original", CV_WINDOW_AUTOSIZE);

    //Trackbar for circle detection tunning.
    //cvCreateTrackbar("Upper","Original",&upperThres,200);
    //cvCreateTrackbar("Center","Original",&centerThres,200);
    cvCreateTrackbar("integralNum","Original",&integralNum,9);
    cvCreateTrackbar("integralDen","Original",&integralDen,1000);
    cvCreateTrackbar("derivativeNum","Original",&derivativeNum,1000);
    cvCreateTrackbar("derivativeDen","Original",&derivativeDen,100);
    cvCreateTrackbar("propNum","Original",&propNum,1000);
    cvCreateTrackbar("propDen","Original",&propDen,100);

    ControllerPID xControl(0, 6, propNum/propDen, integralNum/integralDen, derivativeNum/derivativeDen);
    ControllerPID yControl(0, 6, propNum/propDen, integralNum/integralDen, derivativeNum/derivativeDen);
    xControl.SetDesiredPos_px(368);
    yControl.SetDesiredPos_px(356);

    if(!cap.isOpened())
    {
        cout << "can not open video" << endl;

        return -1;
    }

    averageFPS = getAverageFPS();

    while (INFINITE_LOOP)
    {
        xControl.setGainI(1.0*integralNum/integralDen);
        xControl.setGainD(1.0*derivativeNum/derivativeDen);
        xControl.SetGainP(1.0*propNum/propDen);
        yControl.setGainI(1.0*integralNum/integralDen);
        yControl.setGainD(1.0*derivativeNum/derivativeDen);
        yControl.SetGainP(1.0*propNum/propDen);

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

        processed = ImageProcessing(source, 35, 90, 0, 255, 70, 255, 9);

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
        
        //usleep(100000);

        //Stop process when esc is pressed.
        if(waitKey(30) == 27)
        {
            break;
        }
        //break;
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

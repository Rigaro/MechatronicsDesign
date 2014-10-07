#include "SendSerial.h"
#include "Camera.h"

// Various programmed controllers
#include "controller.h"
#include "controllerpid.h"
#include "tiltcontroller.h"
#include "dualaxiscontroller.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include <iostream>
#include <stdio.h>
#include <vector>

#ifdef UNIX
#include <unistd.h>
#endif

#if defined(WIN32) || defined(WIN64)
#include <Windows.h>
#endif

#define INFINITE_LOOP true
#define BOARD_0_XANG 8
#define BOARD_0_YANG 6

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

    //if (avgFps > FPS_MAX)
    //    avgFps = FPS_MAX;

    return avgFps;
}

int MainProgram()
{
    bool ballOnBoard = false;
    int framesWithoutBall = 0;
    int upperThres = 10;
    int centerThres = 5;
    int minRad = 12, maxRad = 15;
    int xPosBall = 0, yPosBall = 0, prevXPos = 0, prevYPos = 0, radius = 0;
    int xAngle = BOARD_0_XANG, prevXAngle = xAngle;
    int yAngle = BOARD_0_YANG, prevYAngle = yAngle;
    //WITH CONTROLLER LIMIT 1000, X VALUEs
    int xiNum = 27, xiDen = 3, xdNum = 77, xdDen = 15, xpNum = 38, xpDen = 45;
    //WITH CONTROLLER LIMIT 1000, Y VALUEs
    int yiNum = 29, yiDen = 3, ydNum = 77, ydDen = 15, ypNum = 34, ypDen = 41;


    //WITH CONTROLLER LIMIT 100
    //int integralNum = 8, integralDen = 18, derivativeNum = 8, derivativeDen = 28, propNum = 6, propDen = 12;
    //WITH CONTROLLER LIMIT 1000, X VALUEs
    //int xiNum = 27, xiDen = 3, xdNum = 77, xdDen = 15, xpNum = 38, xpDen = 45;
    //WITH CONTROLLER LIMIT 1000, Y VALUEs
    //int yiNum = 29, yiDen = 3, ydNum = 77, ydDen = 15, ypNum = 34, ypDen = 41;

    double timeStuck = 0, timeAtPos = 0;

    Mat source, processed;
    vector<Vec3f> circles;
    //Get video
    VideoCapture cap(CAM_INDEX);

    namedWindow("Trackbars", CV_WINDOW_AUTOSIZE);

    /*
    //Trackbar for circle detection tunning.
    cvCreateTrackbar("Upper","Original",&upperThres,200);
    cvCreateTrackbar("Center","Original",&centerThres,200);
    cvCreateTrackbar("Min Rad","Original",&minRad,30);
    cvCreateTrackbar("Max Rad","Original",&maxRad,50);
    */
    cvCreateTrackbar("xiNum","Trackbars",&xiNum,100);
    cvCreateTrackbar("xiDen","Trackbars",&xiDen,100);
    cvCreateTrackbar("xdNum","Trackbars",&xdNum,100);
    cvCreateTrackbar("xdDen","Trackbars",&xdDen,100);
    cvCreateTrackbar("xpNum","Trackbars",&xpNum,100);
    cvCreateTrackbar("xpDen","Trackbars",&xpDen,100);
    cvCreateTrackbar("yiNum","Trackbars",&yiNum,100);
    cvCreateTrackbar("yiDen","Trackbars",&yiDen,100);
    cvCreateTrackbar("ydNum","Trackbars",&ydNum,100);
    cvCreateTrackbar("ydDen","Trackbars",&ydDen,100);
    cvCreateTrackbar("ypNum","Trackbars",&ypNum,100);
    cvCreateTrackbar("ypDen","Trackbars",&ypDen,100);

    ControllerPID xControl(0, 16, xpNum/xpDen, xiNum/xiDen, xdNum/xdDen);
    ControllerPID yControl(0, 12, ypNum/ypDen, yiNum/yiDen, ydNum/ydDen);
    xControl.setMinimumPositionError(BALL_RAD/2);
    yControl.setMinimumPositionError(BALL_RAD/2);

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

    if(!cap.isOpened())
    {
        cout << "can not open video" << endl;

        return -1;
    }

    averageFPS = getAverageFPS(); // initialize


    // setup our desired path
    vector<Vec4i> path;

    path.push_back(Vec4i(384, 372, 2*BALL_RAD, 2*BALL_RAD)); //Before gate 1
    path.push_back(Vec4i(346, 364, BALL_RAD, BALL_RAD)); //Gate 1
    path.push_back(Vec4i(298, 364, BALL_RAD, BALL_RAD)); //After gate 1
    path.push_back(Vec4i(246, 372, 2*BALL_RAD, 2*BALL_RAD)); // Between gate 1 and 2
    path.push_back(Vec4i(178, 386, BALL_RAD, BALL_RAD)); //Before gate 2
    path.push_back(Vec4i(148, 386, BALL_RAD/2, BALL_RAD/2));  //Gate 2
    path.push_back(Vec4i(116, 382, BALL_RAD, BALL_RAD)); //After gate 2
    path.push_back(Vec4i(108, 317, BALL_RAD, BALL_RAD)); // After gate 2
    path.push_back(Vec4i(130, 280, 3*BALL_RAD, 3*BALL_RAD)); //Midpoint 1
    path.push_back(Vec4i(218, 220, 2*BALL_RAD, 2*BALL_RAD)); // Midpoint 2
    path.push_back(Vec4i(300, 220, 3*BALL_RAD, 3*BALL_RAD));  //Midpoint 3
    path.push_back(Vec4i(434, 172, 3*BALL_RAD, 3*BALL_RAD));  //Midpoint 4
    path.push_back(Vec4i(376, 106, BALL_RAD, BALL_RAD));  //Before gate 3
    path.push_back(Vec4i(348, 106, BALL_RAD/2, BALL_RAD/2));  //Gate 3
    path.push_back(Vec4i(320, 106, BALL_RAD, BALL_RAD));  //After gate 3
    path.push_back(Vec4i(172, 104, BALL_RAD/2, BALL_RAD/2));  //Hole
    int currPosition = 0;


    while (INFINITE_LOOP)
    {
        xControl.setGainI(1.0*xiNum/xiDen);
        xControl.setGainD(1.0*xdNum/xdDen);
        xControl.SetGainP(1.0*xpNum/xpDen);
        yControl.setGainI(1.0*yiNum/yiDen);
        yControl.setGainD(1.0*ydNum/ydDen);
        yControl.SetGainP(1.0*ypNum/ypDen);

        beforeCapTime = (double)getTickCount();

        bool bSuccess = cap.read(source);

        for (int i = 0; i < (int)path.size(); i++)
        {
            Point p = Point(path[i][0], path[i][1]);

            circle(source, p, 4, Scalar(255, 255,0), -1, 8, 0);
        }

        currTime = (double)getTickCount();

        // Get frame time and average FPS
        frameTime = (currTime - beforeCapTime) / getTickFrequency();
        averageFPS = getAverageFPS();
        frameDelta = (currTime - prevTime) / getTickFrequency();

        printf("Frame time: %0.4f, FPS: %0.3f, Average FPS: %0.4f, Frame delta: %0.2f\n",
            frameTime, 1/frameTime, averageFPS, frameDelta);

        if (!bSuccess)
        {
            cout << "Can't read frame from video" << endl;
            break;
        }

        processed = ImageProcessing(source, 35, 90, 0, 255, 70, 255, 9);

        //Get circles from processed image.
        HoughCircles(processed, circles, CV_HOUGH_GRADIENT, 1, processed.rows/8, 
                     upperThres, centerThres, minRad, maxRad);

        Vec4i newDesPos = path[currPosition];
        Point newXY = Point(newDesPos[0], newDesPos[1]);
        printf("Num pos: %d, curPos: %d,  New des position - x: %d, y: %d\n", path.size(), currPosition, newXY.x, newXY.y);

        xControl.setDesiredPos_px(newXY.x);
        yControl.setDesiredPos_px(newXY.y);

        circle(source, newXY, 4, Scalar(255,0,0), -1, 8, 0);

        ballOnBoard = circles.size() != 0;

        if (ballOnBoard) 
        {
            framesWithoutBall = 0;

            prevXPos = xPosBall;
            prevYPos = yPosBall;
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

            xAngle = xControl.positionControl(xPosBall,frameDelta);
            yAngle = yControl.positionControl(yPosBall,frameDelta);


            //perform a check to see if the ball has been stuckj in a certain position
            //for longer than 3 seconds. if so, jerk it so that it moves

            //printf("Time stuck: %0.2f\n", timeStuck/getTickFrequency());
            if (abs(prevXPos - xPosBall) <= xControl.getMinumumPositionError() &&
                    abs(prevYPos - yPosBall) <= yControl.getMinumumPositionError())
            {
                timeStuck += currTime - prevTime;

                if (timeStuck/getTickFrequency() > 4)
                {
                    if (xAngle == xControl.GetOutputMax())
                        xAngle = BOARD_0_XANG/4;

                    else
                        xAngle = 1.25*BOARD_0_XANG;

                    if (yAngle == yControl.GetOutputMax())
                        yAngle = BOARD_0_YANG/4;

                    else
                        yAngle = 1.25*BOARD_0_YANG;

                    timeStuck = 0;
                }

            }
            else
                timeStuck = 0;

            // If we're at the desired position, increment the path counter so we try
            // to go to the next point starting with the next frame
            if (xControl.atDesiredPosition() && yControl.atDesiredPosition())
            {
                timeAtPos += currTime - prevTime;

                if (timeAtPos/getTickFrequency() > 0)
                {
                    currPosition++;

                    xControl.setMinimumPositionError(newDesPos[2]);
                    yControl.setMinimumPositionError(newDesPos[3]);

                    if (currPosition >= (int)path.size())
                    {
                        cout << "AT END POINT. Restarting" << endl;
                        currPosition = 0;
                    }

                    timeAtPos = 0;
                }
            }
            else
                timeAtPos = 0;
        }
        else if (framesWithoutBall == 3)
        {
            cout << 'reversing angle' << endl;
            int xAngleDiff = BOARD_0_XANG - xAngle;
            int yAngleDiff = BOARD_0_YANG - yAngle;
            xAngle = BOARD_0_XANG;
            yAngle = BOARD_0_YANG;
            framesWithoutBall = 0;
        }
        else
        {
            // Count how many frames pass without detecting the ball
            framesWithoutBall++;
        }
        printf("Frames passed without detecting the balL: %d\n", framesWithoutBall);
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

        prevTime = currTime;

        //Stop process when esc is pressed.
        if(waitKey(30) == 27)
        {
            cout << "ESC PRESSED" << endl;
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
    int yAngle = 91;

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


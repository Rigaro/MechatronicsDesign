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
#define BOARD_0_XANG 3
#define BOARD_0_YANG 2

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
    int minRad = 12, maxRad = 15;
    int xPosBall = 0, yPosBall = 0, radius = 0;
    int xAngle = BOARD_0_XANG, prevXAngle = xAngle;
    int yAngle = BOARD_0_YANG, prevYAngle = yAngle;
    int integralNum = 8, integralDen = 18, derivativeNum = 8, derivativeDen = 28, propNum = 6, propDen = 12;
    //WITH CONTROLLER LIMIT 100
    //int integralNum = 8, integralDen = 18, derivativeNum = 8, derivativeDen = 28, propNum = 6, propDen = 12;
    //WITH CONTROLLER LIMIT 1000
    //int integralNum = 40, integralDen = 5, derivativeNum = 23, derivativeDen = 5, propNum = 14, propDen = 5;

    Mat source, processed;
    vector<Vec3f> circles;
    //Get video
    VideoCapture cap(CAM_INDEX);

    namedWindow("Original", CV_WINDOW_AUTOSIZE);

    /*
    //Trackbar for circle detection tunning.
    cvCreateTrackbar("Upper","Original",&upperThres,200);
    cvCreateTrackbar("Center","Original",&centerThres,200);
    cvCreateTrackbar("Min Rad","Original",&minRad,30);
    cvCreateTrackbar("Max Rad","Original",&maxRad,50);
    */
    cvCreateTrackbar("integralNum","Original",&integralNum,100);
    cvCreateTrackbar("integralDen","Original",&integralDen,100);
    cvCreateTrackbar("derivativeNum","Original",&derivativeNum,100);
    cvCreateTrackbar("derivativeDen","Original",&derivativeDen,100);
    cvCreateTrackbar("propNum","Original",&propNum,100);
    cvCreateTrackbar("propDen","Original",&propDen,100);

    ControllerPID xControl(0, 20, propNum/propDen, integralNum/integralDen, derivativeNum/derivativeDen);
    ControllerPID yControl(0, 20, propNum/propDen, integralNum/integralDen, derivativeNum/derivativeDen);


    /*
    // Tilt-based controller
    // WORKING: BOTH AT 500ms delay, 100ms tilt action
    int xTiltDelay = 705;
    int xTiltActionTime = 45;
    int yTiltDelay = 500;
    int yTiltActionTime = 100;
    cvCreateTrackbar("xTiltDelay", "Original", &xTiltDelay, 2000);
    cvCreateTrackbar("xTiltActionTime", "Original", &xTiltActionTime, 2000);
    cvCreateTrackbar("yTiltDelay", "Original", &yTiltDelay, 2000);
    cvCreateTrackbar("yTiltActionTime", "Original", &yTiltActionTime, 2000);

    TiltController xControl = TiltController(0, 6, BOARD_0_XANG, xTiltDelay, xTiltActionTime, minRad);
    TiltController yControl = TiltController(0, 4, BOARD_0_YANG, yTiltDelay, yTiltActionTime, minRad);

    // Dual axis control
    DualAxisController control = DualAxisController(0, 2, BOARD_0_XANG, 0.2, 0.05, BALL_RAD);
    control.setXYDesiredPosition_px(368, 356);

    xControl.setDesiredPos_px(172);
    yControl.setDesiredPos_px(78);
    */

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

    /*
    // setup our desired path
    vector<Point> path;

    //path.push_back(Point(526, 434)); // START POINT
    path.push_back(Point(464, 366));
    path.push_back(Point(360, 352));
    path.push_back(Point(268, 382));
    path.push_back(Point(158, 370)); // GATE 1?
    path.push_back(Point(88, 340));
    path.push_back(Point(202, 238));
    path.push_back(Point(474, 188));
    path.push_back(Point(448, 82));
    path.push_back(Point(364, 74));
    path.push_back(Point(256, 78));
    path.push_back(Point(172, 78));
    int currPosition = 0;
    */

    while (INFINITE_LOOP)
    {
        xControl.setGainI(1.0*integralNum/integralDen);
        xControl.setGainD(1.0*derivativeNum/derivativeDen);
        xControl.SetGainP(1.0*propNum/propDen);
        yControl.setGainI(1.0*integralNum/integralDen);
        yControl.setGainD(1.0*derivativeNum/derivativeDen);
        yControl.SetGainP(1.0*propNum/propDen);

        /*
        xControl.setTiltDelay(xTiltDelay);
        xControl.setTiltActionTime(xTiltActionTime);
        yControl.setTiltDelay(yTiltDelay);
        yControl.setTiltActionTime(yTiltActionTime);
        */
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
                     upperThres, centerThres, minRad, maxRad);

        /*
        Point newPos = path[currPosition];
        printf("Num pos: %d, curPos: %d,  New des position - x: %d, y: %d\n", path.size(), currPosition, newPos.x, newPos.y);
        */
        xControl.setDesiredPos_px(300);
        yControl.setDesiredPos_px(300);

        printf("x des pos: %d, y des pos: %d\n", xControl.GetDesiredPos_px(),
               yControl.GetDesiredPos_px());

        //circle(source, newPos, 4, Scalar(255,0,0), -1, 8, 0);

        ballOnBoard = circles.size() != 0;

        if (ballOnBoard) 
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

            // SINGLE AXIS CONTROLLERS (1 control per axis)
            //xControl.setTiltActionTime(frameDelta*1000 - 100);
            //yControl.setTiltActionTime(frameDelta*1000 - 100);

            //if (yAngle == BOARD_0_YANG)
                xAngle = xControl.positionControl(xPosBall,frameDelta);
            /* Prevent Y from changing if X is not at the zero position, to
            'emulate' dual axis control, this should prevent the ball
            shooting off in a diagonal if they both change at once. This
            only happens if Y is also at the zero position (or it'll be
            stuck on a angle!) */
            //if (xAngle == BOARD_0_XANG)
                yAngle = yControl.positionControl(yPosBall,frameDelta);
        }
        /*
        else
        {
            // If using tilt control, we want the board to be flat if we don't
            // detect the ball.
            xAngle = BOARD_0_XANG;
            yAngle = BOARD_0_YANG;
        }
        */
        //Send x data
        SendSerial(xAngle, 'x', PORT_0);
        //Send y data
        SendSerial(yAngle, 'y', PORT_0);

        printf("C X ang: %d, D X ang: %d, x pos: %d, x error: %d\n", xAngle,
            prevXAngle, xPosBall, xControl.GetCurrentError());

        printf("C Y ang: %d, D Y ang: %d, y pos: %d, y error: %d\n", yAngle,
            prevYAngle, yPosBall, yControl.GetCurrentError());

        /*
        // If we're at the desired position, increment the path counter so we try
        // to go to the next point starting with the next frame
        if (xControl.atDesiredPosition() && yControl.atDesiredPosition())
        {
            currPosition++;

            if (currPosition >= (int)path.size())
            {
                cout << "AT END POINT. EXITING" << endl;
                break;
            }
        }
        */

        imshow("Original", source);
        imshow("Thresh", processed);
        
        //usleep(100000);

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


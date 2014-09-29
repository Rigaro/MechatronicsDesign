#include "SendSerial.h"
#include "Camera.h"
#include "controller.h"
#include "controllerpid.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include <iostream>
#include <stdio.h>

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
	static double startTime = getTickCount();
	static long frameCount = 0;

	double currTime = getTickCount();
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
    int xAngle = BOARD_0_XANG;
    int yAngle = BOARD_0_YANG;

	/* 
	To determine our sample frequency, we must determine the frame rate of the
	camera, as this is the frequency at which we receive updates.
	i.e. VideoCapture.read is a blocking method, so we will only process when
	we have a frame available. Therefore, the frame rate dictates our sample
	frequency.

	In ideal conditions, our FPS sits on 24.
	*/
	double currTime = 0, frameTime = 0, beforeCapTime = 0;
	double averageFPS = 0;

    Mat source, processed;
    vector<Vec3f> circles;
    //Get video
    VideoCapture cap(CAM_INDEX);

    ControllerPID xControl(0,16,0.1,0.1,0.01,100);
    ControllerPID yControl(0,16,0.1,0.1,0.01,100);
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

		// Get frame time and total running time
		frameTime = (currTime - beforeCapTime) / getTickFrequency();
		averageFPS = getAverageFPS();

		printf("Frame time: %0.4f, FPS: %0.3f, Average FPS: %0.4f\n", 
			frameTime, 1/frameTime, averageFPS);

        if (!bSuccess)
        {
            cout << "Can't read frame from video" << endl;
            break;
        }

        processed = ImageProcessing(source, thresh, 9);

        //Get circles from processed image.
        HoughCircles(processed, circles, CV_HOUGH_GRADIENT, 1, processed.rows/8, upperThres, centerThres, 15, 25);

        ballOnBoard = circles.size() != 0;

        if(ballOnBoard) 
		{
            source = GetBallPosition(&xPosBall, &yPosBall, &radius, circles, source);
            xAngle = xControl.PositionControl(xPosBall);
            yAngle = yControl.PositionControl(yPosBall);
        }


        //Send x data
        SendSerial(xAngle, 'x', PORT_0);
        //Send y data
        SendSerial(yAngle, 'y', PORT_0);

        /*cout << "x ang: " << xAngle << endl;
        cout << "x pos: " << xPosBall << endl;
        cout << "x error: " << xControl.GetCurrentError() << endl;
        cout << "y ang: " << yAngle << endl;
        cout << "y pos: " << yPosBall << endl;
        cout << "y error: " << yControl.GetCurrentError() << endl;*/

        imshow("Original", source);
        imshow("Thresh", processed);

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

#include "controllerpid.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

ControllerPID::ControllerPID() : Controller()
{
    gainI = 0;
    gainD = 0;
    samplingFreq = 10;
    integral = 0;
    derivative = 0;
    previousError = 0;
}

ControllerPID::ControllerPID(int outputMin_deg, int outputMax_deg, 
                             double gainP, double gainI, double gainD)
                            : Controller(outputMin_deg, outputMax_deg, gainP)
{
    this->gainI = gainI;
    this->gainD = gainD;

    samplingFreq = 10;
    integral = 0;
    derivative = 0;
}

void ControllerPID::computeError()
{
    previousError = error;
    error = desPos_px - curPos_px;
}

/**
Performs the integral correction for the current position.
@return controller proportional correction as integer.
*/
double ControllerPID::integralCorrection(double delta)
{
    integral = integral + gainI*error*delta;
    if(integral > UPPERLIMIT/2)
        integral = UPPERLIMIT/2;
    else if(integral < LOWERLIMIT/2)
        integral = LOWERLIMIT/2;

    return integral;
}

/**
Performs the derivative correction for the current position.
@return controller proportional correction as integer.
*/
double ControllerPID::derivativeCorrection(double delta)
{
    double errorDiff = error - previousError;
    double absErrorDiff = abs(errorDiff);

    if (errorDiff > 0 && absErrorDiff > minimumPositionError)
        derivative = gainD * errorDiff / delta;
    else
        derivative = 0;

    return derivative;
}

/**
Performs PID position control, uses sampling frequency for frame delta
calculation
*/
int ControllerPID::positionControl(int desPox_px, int curPos_px)
{
    setDesiredPos_px(desPox_px);
    double frameDelta = 1 / samplingFreq;
    return positionControl(curPos_px, frameDelta);
}

/**
Performs PID position control that sets the desired position.
The output is normalized to work within the application's requirements.
@return normalized control signal.
*/
int ControllerPID::positionControl(int desPos_px, int curPos_px, double frameDelta)
{
    setDesiredPos_px(desPos_px);
    
    return positionControl(curPos_px, frameDelta);
}

/**
Performs PID position control.
The output is normalized to work within the application's requirements.
@return normalized control signal.
*/
int ControllerPID::positionControl(int curPos_px, double frameDelta)
{
    setCurrentPos_px(curPos_px);
    computeError();

    double controlSignal = proportionalCorrection() + \
        integralCorrection(frameDelta) + derivativeCorrection(frameDelta);

    controlSignal = clampSaturation(controlSignal);

    //if(abs(error) <= BALL_RAD)
    //    controlSignal = 0;

    //printf("Derivative: %0.2f, Integral: %0.2f, Prop: %0.2f\n",
     //      derivative, integral, GetGainP()*error);

    return normalizeData(controlSignal);
}

void ControllerPID::setGainI(double gainI)
{
    this->gainI = gainI;
}

void ControllerPID::setGainD(double gainD)
{
    this->gainD = gainD;
}

//Getters
double ControllerPID::getGainI()
{
    return gainI;
}

//Getters
double ControllerPID::getGainD()
{
    return gainD;
}

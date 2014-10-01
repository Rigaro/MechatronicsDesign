#include "controllerpid.h"
#include <iostream>
#include <stdio.h>

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

void ControllerPID::ComputeError()
{
    previousError = error;
    error = desPos_px - curPos_px;
}

/**
Performs the integral correction for the current position.
@return controller proportional correction as integer.
*/
double ControllerPID::IntegralCorrection(double delta)
{
    integral = integral + gainI*error*delta;

    return integral;
}

/**
Performs the derivative correction for the current position.
@return controller proportional correction as integer.
*/
double ControllerPID::DerivativeCorrection(double delta)
{
    double errorDif = error - previousError;
    double absErrorDif;

    if(errorDif >= 0)
        absErrorDif = errorDif;
    else
        absErrorDif = -1*errorDif;

    if( absErrorDif > BALL_RAD)
        derivative = gainD * errorDif / delta;
    else
        derivative = 0;

    return derivative;
}

/**
Performs PID position control, uses sampling frequency for frame delta
calculation
*/
int ControllerPID::PositionControl(int desPox_px, int curPos_px)
{
    double frameDelta = 1 / samplingFreq;
    return PositionControl(desPos_px, frameDelta);
}

/**
Performs PID position control that sets the desired position.
The output is normalized to work within the application's requirements.
@return normalized control signal.
*/
int ControllerPID::PositionControl(int desPos_px, int curPos_px, double frameDelta)
{
    SetDesiredPos_px(desPos_px);
    
    return PositionControl(curPos_px, frameDelta);
}

/**
Performs PID position control.
The output is normalized to work within the application's requirements.
@return normalized control signal.
*/
int ControllerPID::PositionControl(int curPos_px, double frameDelta)
{
    SetCurrentPos_px(curPos_px);
    ComputeError();

    double controlSignal = ProportionalCorrection() + \
        IntegralCorrection(frameDelta) + DerivativeCorrection(frameDelta);

    controlSignal = ClampSaturation(controlSignal);

    printf("Derivative: %0.2f, Integral: %0.2f, Prop: %0.2f\n",
           derivative, integral, GetGainP()*error);

    return NormalizeData(controlSignal);
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

#include "controllerpid.h"

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
    error = this->desPos_px - this->curPos_px;
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
    derivative = gainD * (error - previousError) / delta;

    return derivative;
}

/**
Performs PID position control, uses sampling frequency for frame delta
calculation
*/
int ControllerPID::PositionControl(int desPox_px, int curPos_px)
{
    double frameDelta = 1 / samplingFreq;
    return PositionControl(desPos_px, curPos_px);
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

    controlSignal = ProportionalCorrection() + IntegralCorrection() + DerivativeCorrection();
    controlSignal = ClampSaturation(controlSignal);

    return NormalizeData(controlSignal);
}

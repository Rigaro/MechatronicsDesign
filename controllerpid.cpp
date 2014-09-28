#include "controllerpid.h"

ControllerPID::ControllerPID() : Controller()
{
    gainI = 0;
    gainD = 0;
    samplingFreq = 100;
    integral = 0;
    derivative = 0;
}

ControllerPID::ControllerPID(int outputMin_deg, int outputMax_deg, double gainP,
                             double gainI, double gainD, double samplingFreq)
                            : Controller(outputMin_deg, outputMax_deg, gainP)
{
    this->gainI = gainI;
    this->gainD = gainD;
    if(samplingFreq > 0)
        this->samplingFreq = samplingFreq;
    integral = 0;
    derivative = 0;
}

//Performs the integral correction for the current position.
//If the correction is greater or lower than the controller's limits,
//the value is set to that of the limit.
//@return controller proportional correction as integer.
double ControllerPID::IntegralCorrection()
{
    integral += error_k0/samplingFreq;
    double correctionI = gainI * integral;
    if(correctionI > UPPERLIMIT)
        correctionI = UPPERLIMIT;
    else if(correctionI < LOWERLIMIT)
        correctionI = LOWERLIMIT;
    return correctionI;
}

//Performs the derivative correction for the current position.
//If the correction is greater or lower than the controller's limits,
//the value is set to that of the limit.
//@return controller proportional correction as integer.
double ControllerPID::DerivativeCorrection()
{
    derivative = (error_k0 - error_k1)*samplingFreq;
    double correctionD = gainD * derivative;
    if(correctionD > UPPERLIMIT)
        correctionD = UPPERLIMIT;
    else if(correctionD < LOWERLIMIT)
        correctionD = LOWERLIMIT;
    return correctionD;
}

//Performs PID position control that sets the desired position.
//The output is normalized to work within the application's requirements.
//@return normalized control signal.
int ControllerPID::PositionControl(int desPos_px, int curPos_px)
{
    SetCurrentPos_px(curPos_px);
    SetDesiredPos_px(desPos_px);
    ComputeError();
    controlSignal = ProportionalCorrection() + IntegralCorrection() + DerivativeCorrection();
    if(controlSignal > UPPERLIMIT)
        controlSignal = UPPERLIMIT;
    else if(controlSignal < LOWERLIMIT)
        controlSignal = LOWERLIMIT;
    return NormalizeData(controlSignal);
}

//Performs PID position control.
//The output is normalized to work within the application's requirements.
//@return normalized control signal.
int ControllerPID::PositionControl(int curPos_px)
{
    SetCurrentPos_px(curPos_px);
    ComputeError();
    controlSignal = ProportionalCorrection() + IntegralCorrection() + DerivativeCorrection();
    if(controlSignal > UPPERLIMIT)
        controlSignal = UPPERLIMIT;
    else if(controlSignal < LOWERLIMIT)
        controlSignal = LOWERLIMIT;
    return NormalizeData(controlSignal);
}

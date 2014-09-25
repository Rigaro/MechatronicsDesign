/*
 * @author Ricardo Garcia Rosas
 *
 * @section DESCRIPTION
 *
 * Basic proportional position controller. Tilt table as main application.
 * This class should be extended to implement PID, linearized, etc. controllers.
 *
 */
#include "controller.h"

//Default contructor
Controller::Controller()
{
    errorNew = 0;
    errorPast = 0;
    gainP = 0;
    desiredPos = 0;
    currentPos = 0;
    output = 0;
    outputMin = 0;
    outputMax = 0;
}

//Initializes controller limits and gain.
//@param controller output lower limit.
//@param controller output upper limit.
//@param controller proportional gain.
Controller::Controller(int outputMin, int outputMax, double gainP)
{
    errorNew = 0;
    errorPast = 0;
    this->gainP = gainP;
    desiredPos = 0;
    currentPos = 0;
    output = 0;
    this->outputMin = outputMin;
    this->outputMax = outputMax;
}

//Computes the error between the desired and current position.
void Controller::ComputeError()
{
    errorPast = errorNew;
    errorNew = desiredPos - posNew;
}

//Performs the proportional correction for the current position.
//If the correction is greater or lower than the controller's limits,
//the value is set to that of the limit.
//@return controller proportional correction as integer.
double Controller::ProportionalCorrection()
{
    double correctionP = gainP * errorNew;
    if(correctionP > UPPERLIMIT)
        correctionP = UPPERLIMIT;
    else if(correctionP < LOWERLIMIT)
        correctionP = LOWERLIMIT;
    return correctionP;
}

//Performs proportional position control that sets the desired position.
//The output is normalized to work within the application's requirements.
//@return normalized control signal.
int Controller::PositionControl(int desiredPos, int posPixel)
{
    SetPosPixel(posPixel);
    SetDesiredPos(desiredPos);
    ComputeError();
    output = ProportionalCorrection();
    if(output > outputMax)
        output = outputMax;
    else if(output < outputMin)
        output = outputMin;
    return output;
}

//Performs proportional position control.
//The output is normalized to work within the application's requirements.
//@return normalized control signal.
int Controller::PositionControl(int currentPos)
{
    transformPos(posPixel);
    ComputeError();
    output = ProportionalCorrection();
    if(output > UPPERLIMIT)
        output = UPPERLIMIT;
    else if(output < LOWERLIMIT)
        output = LOWERLIMIT;
    return output;
}

void transformPos(int posPixel)
{
    setPosPast(posNew);
    setPosNew(posPixel*TSCALAR+TCONST);
}

//Normalizes control data to the output range.
//@param data to be normalized.
//@return normalized control data.
int Controller::NormalizeData(int data)
{
    return (data - LOWERLIMIT)*(outputMax - outputMin)/(UPPERLIMIT - LOWERLIMIT);
}

//Setters
void Controller::SetDesiredPos(int desiredPos)
{
    this->desiredPos = desiredPos;
}

void Controller::SetPosNew(int posNew)
{
    this->posNew = posNew;
}

void Controller::SetPosPast(int posPast)
{
    this->posPast = posPast;
}

void Controller::SetPosPixel(int posPixel)
{
    this->posPixel = posPixel;
}

void Controller::SetGainP(double gainP)
{
    this->gainP = gainP;
}

void Controller::SetOutputMax(int outputMax)
{
    this->outputMax = outputMax;
}

void Controller::SetOutputMin(int outputMin)
{
    this->outputMin = outputMin;
}

//Getters
double Controller::GetGainP()
{
    return gainP;
}

int Controller::GetOutputMax()
{
    return outputMax;
}

int Controller::GetOutputMin()
{
    return outputMin;
}

int Controller::GetDesiredPos()
{
    return desiredPos;
}

int Controller::GetCurrentPos()
{
    return currentPos;
}

int Controller::GetErrorNew()
{
    return errorNew;
}

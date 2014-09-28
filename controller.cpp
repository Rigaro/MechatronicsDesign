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

//Default constructor
Controller::Controller()
{
    error_k0 = 0;
    error_k1 = 0;
    desPos_px = 0;
    desPos_mm = 0.0;
    curPos_px = 0;
    curPos_mm = 0.0;
    gainP = 0.0;
    outputMin_deg = 0.0;
    outputMax_deg = 0.0;
    controlSignal = 0.0;
}

//Initializes controller limits and gain.
//@param controller output lower limit.
//@param controller output upper limit.
//@param controller proportional gain.
Controller::Controller(int outputMin_deg, int outputMax_deg, double gainP)
{
    error_k0 = 0;
    error_k1 = 0;
    desPos_px = 0;
    desPos_mm = 0.0;
    curPos_px = 0;
    curPos_mm = 0.0;
    this->gainP = gainP;
    this->outputMin_deg = outputMin_deg;
    this->outputMax_deg = outputMax_deg;
    controlSignal = 0;
}

//Computes the error between the desired and current position.
void Controller::ComputeError()
{
    error_k1 = error_k0;
    error_k0 = desPos_px - curPos_px;
}

//Performs the proportional correction for the current position.
//If the correction is greater or lower than the controller's limits,
//the value is set to that of the limit.
//@return controller proportional correction as integer.
double Controller::ProportionalCorrection()
{
    double correctionP = gainP * error_k0;
    if(correctionP > UPPERLIMIT)
        correctionP = UPPERLIMIT;
    else if(correctionP < LOWERLIMIT)
        correctionP = LOWERLIMIT;
    return correctionP;
}

//Performs proportional position control that sets the desired position.
//The output is normalized to work within the application's requirements.
//@return normalized control signal.
int Controller::PositionControl(int desPos_px, int curPos_px)
{
    SetCurrentPos_px(curPos_px);
    SetDesiredPos_px(desPos_px);
    ComputeError();
    controlSignal = ProportionalCorrection();
    if(controlSignal > UPPERLIMIT)
        controlSignal = UPPERLIMIT;
    else if(controlSignal < LOWERLIMIT)
        controlSignal = LOWERLIMIT;
    return NormalizeData(controlSignal);
}

//Performs proportional position control.
//The output is normalized to work within the application's requirements.
//@return normalized control signal.
int Controller::PositionControl(int curPos_px)
{
    SetCurrentPos_px(curPos_px);
    ComputeError();
    controlSignal = ProportionalCorrection();
    if(controlSignal > UPPERLIMIT)
        controlSignal = UPPERLIMIT;
    else if(controlSignal < LOWERLIMIT)
        controlSignal = LOWERLIMIT;
    return NormalizeData(controlSignal);
}

/*
void transformPos(int posPixel)
{
    SetPosPast(posNew);
    SetPosNew(posPixel*TSCALAR+TCONST);
}
*/

//Normalizes control data to the output range.
//@param data to be normalized.
//@return normalized control data.
int Controller::NormalizeData(int data)
{
    return (data - LOWERLIMIT)*(outputMax_deg - outputMin_deg)/(UPPERLIMIT - LOWERLIMIT);
}

//Setters
void Controller::SetDesiredPos_px(int desPos_px)
{
    this->desPos_px = desPos_px;
}

void Controller::SetDesiredPos_mm(double desPos_mm)
{
    this->desPos_mm = desPos_mm;
}


void Controller::SetCurrentPos_px(int curPos_px)
{
    this->curPos_px = curPos_px;
}

void Controller::SetCurrentPos_mm(double desPos_mm)
{
    this->desPos_mm = desPos_mm;
}

void Controller::SetGainP(double gainP)
{
    this->gainP = gainP;
}

void Controller::SetOutputMax(int outputMax_deg)
{
    this->outputMax_deg = outputMax_deg;
}

void Controller::SetOutputMin(int outputMin_deg)
{
    this->outputMin_deg = outputMin_deg;
}

//Getters
double Controller::GetGainP()
{
    return gainP;
}

int Controller::GetOutputMax()
{
    return outputMax_deg;
}

int Controller::GetOutputMin()
{
    return outputMin_deg;
}

int Controller::GetDesiredPos_px()
{
    return desPos_px;
}

double Controller::GetDesiredPos_mm()
{
    return desPos_mm;
}

int Controller::GetCurrentPos_px()
{
    return curPos_px;
}

double Controller::GetCurrentPos_mm()
{
    return curPos_mm;
}

int Controller::GetCurrentError()
{
    return error_k0;
}

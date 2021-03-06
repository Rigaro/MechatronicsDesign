#include "fscontroller.h"

void FSController::initialize()
{
    state1Gain = 0;
    state2Gain = 0;

    previousObservedStateValue = 0;
}

FSController::FSController()
{
    initialize();
}

FSController::FSController(int outputMin_deg, int outputMax_deg,
    double state1Gain, double state2Gain) : Controller()
{
    initialize();

    SetOutputMax(outputMax_deg);
    SetOutputMin(outputMin_deg);

    this->state1Gain = state1Gain;
    this->state2Gain = state2Gain;

    previousObservedStateValue = 0;
}

int FSController::positionControl(int desPos_px, int currPos_px, double frameDelta)
{
    setDesiredPos_px(desPos_px);

    return positionControl(currPos_px, frameDelta);
}

int FSController::positionControl(int currPos_px, double frameDelta)
{
    return 0;
}



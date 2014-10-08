#include "tiltcontroller.h"
#include "opencv/cv.h"

TiltController::TiltController() : Controller()
{
    tiltDelay = 0;
    tiltActionTime = 0;
    tiltStartTick = 0;
    outputZero = 0;
}

/**
Initializes the tilt controller. We take the maximum and minimum output angles,
the delay between each tilt we perform and how long each tilt should last for.

@param outputMin_deg The minimum angle we can tilt to (in degrees)
@param outputMax_deg The maximum angle we can titl to (in deg)
@param tiltDelay The delay between tilts (in milliseconds)
@param tiltActionTime How long the tilt should last for (in milliseconds)
*/
TiltController::TiltController(int outputMin_deg, int outputMax_deg, 
                               int outputZero,
                               double tiltDelay, double tiltActionTime,
                               double minimumPositionError) : TiltController()
{
    SetOutputMax(outputMax_deg);
    SetOutputMin(outputMin_deg);

    this->minimumPositionError = minimumPositionError;
    this->tiltDelay = tiltDelay;
    this->tiltActionTime = tiltActionTime;
    this->outputZero = outputZero;
}

/**
Wraps PositionControl(desPox_px, curPos_px)
*/
int TiltController::positionControl(int desPox_ps, int curPos_px, 
                                    double minimumPositionError)
{
    setMinimumPositionError(minimumPositionError);

    return positionControl(desPox_ps, curPos_px);
}

/**
Wraps PositionControl(curPos_px)

@param desPos_px The desired position (in pixels)
@param curPos_px The current position (in pixels)
*/
int TiltController::positionControl(int desPos_px, int curPos_px)
{
    setDesiredPos_px(desPos_px);

    return positionControl(curPos_px);
}

/**
Performs our position control for this axis. The goal is to tilt the axis to
outputMax_deg (or outputMin_deg) for tiltActionTime milliseconds and back to
zero, and then wait tiltDelay milliseconds, repeating until
desPos_px ~= curPos_px. We use the set error range to determine at which point
we stop attempting to tilt the ball. i.e If the ball position error is within
the set error range, we do nothing.

@param curPos_px The current position (in pixels)
*/
int TiltController::positionControl(int curPos_px)
{
    this->curPos_px = curPos_px;
    computeError();

    double controlSignal = 0;

    if (abs(error) <= minimumPositionError)
    {
        controlSignal = outputZero;
    }
    else
    {
        controlSignal = (double)GetTilt();
    }

    return (int)clampSaturation(controlSignal);
}

/**
Gets the tilt angle we should tilt at, using time based logic to determine if
we should tilt or not.
*/
int TiltController::GetTilt()
{
    double currentTick = (double)cv::getTickCount();
    double diff = (currentTick - tiltStartTick) * 1000 / cv::getTickFrequency();

    /* Diff is now the time elapsed in milliseconds since the start of the last
    tilt */

    int tiltAngle = outputZero;
    // If diff <= tiltActionTime, we are still inside a tilt action. So we tilt!
    if (diff <= tiltActionTime)
    {
        tiltAngle = DetermineTiltAngle();
    }
    // If diff > tiltActionTime, but < tiltActionTime + tiltDelay, we are between
    // tilts. We _NEED_ to explicitly return this so that we go back to the zero
    // position after a tilt.
    else if (diff > tiltActionTime && diff <= tiltActionTime + tiltDelay) {
        tiltAngle = outputZero;
    }
    // If diff > tiltActionTime + tiltDelay, it's time to start tilting again
    else if (diff > tiltActionTime + tiltDelay)
    {
        tiltAngle = DetermineTiltAngle();

        // Update the tilt start tick to the current tick so we're back to a 0
        // diff for future calculations
        tiltStartTick = currentTick;
    }

    return tiltAngle;
}

/**
Determines which angle we should tilt at. If the ball error is > 0, we should
tilt to the minimum. If the ball error is < 0, we should tilt to the maximum.
*/
int TiltController::DetermineTiltAngle()
{
    if (abs(error) <= minimumPositionError*2)
    {
        if (error > 0)
            return GetOutputMin()+1;
        else
            return GetOutputMax()-1;
    }
    else if (error > 0)
        return GetOutputMin();
    else
        return GetOutputMax();

}


/* Setters */
void TiltController::setTiltDelay(double tiltDelay)
{
    this->tiltDelay = tiltDelay;
}

void TiltController::setTiltActionTime(double tiltActionTime)
{
    this->tiltActionTime = tiltActionTime;
}

/* Getters */


double TiltController::getTiltDelay()
{
    return this->tiltDelay;
}

double TiltController::getTiltActionTime()
{
    return this->tiltActionTime;
}

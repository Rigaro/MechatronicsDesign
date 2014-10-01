#include "tiltcontroller.h"
#include "opencv/cv.h"

TiltController::TiltController() : Controller()
{
    tiltDelay = 0;
    tiltActionTime = 0;
    minimumPositionError = 0;
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
                               double tiltDelay, double tiltActionTime,
                               double minimumPositionError) : TiltController()
{
    SetOutputMax(outputMax_deg);
    SetOutputMin(outputMin_deg);

    this->minimumPositionError = minimumPositionError;
    this->tiltDelay = tiltDelay;
    this->tiltActionTime = tiltActionTime;
}

/**
Allows us to pass the minimum position error, overriding the one set in the
constructor.
*/
int TiltController::PositionControl(int desPox_ps, int curPos_px, 
                                    double minimumPositionError)
{
    setMinimumPositionError(minimumPositionError);

    return PositionControl(desPox_ps, curPos_px);
}

/**
Performs our position control for this axis. The goal is to tilt the axis to
outputMax_deg (or outputMin_deg) for tiltActionTime milliseconds and then wait
tiltDelay milliseconds, repeating until desPos_px ~= curPos_px. We use the set
error range to determine at which point we stop attempting to tilt the ball.
i.e If the ball position error is within the set error range, we do nothing.

@param desPos_px The desired position (in pixels)
@param curPos_px The current position (in pixels)
*/
int TiltController::PositionControl(int desPos_px, int curPos_px)
{

}


/* Setters */
void TiltController::setMinimumPositionError(double minimumPositionError)
{
    this->minimumPositionError = minimumPositionError;
}

/* Getters */
double TiltController::getMinumumPositionError()
{
    return this->minimumPositionError;
}

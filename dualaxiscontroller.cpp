/*
dualaxiscontroller.cpp

A tilt based controller that controls both axis, allowing us to only move one
at a time.
*/

#include "dualaxiscontroller.h"

DualAxisController::DualAxisController()
{
    xyDesPos_px = cv::Point(0, 0);
    xyCurPos_px = cv::Point(0, 0);
    error = cv::Point(0, 0);

    controllingX = false;
    controllingY = false;
}

DualAxisController::DualAxisController(int outputMin_deg, int outputMax_deg,
    int outputZero,
    double tiltDelay, double tiltActionTime,
    double minimumPositionError) : DualAxisController()
{
    this->outputMin_deg = outputMin_deg;
    this->outputMax_deg = outputMax_deg;

    this->minimumPositionError = minimumPositionError;
    this->tiltDelay = tiltDelay;
    this->tiltActionTime = tiltActionTime;
    this->outputZero = outputZero;
}

void DualAxisController::ComputeError()
{
    error.x = xyDesPos_px.x - xyCurPos_px.x;
    error.y = xyDesPos_px.y - xyCurPos_px.y;
}

/**
Allows positions to be passed in as ints (pixels). Wraps AngleControl(int, int)

@param desPos_x The desired X position (in pixels)
@param desPos_y The desired Y Position (in pixels)
@param curPos_x The current X position (in pixels)
@param curPos_y The current Y position (in pixels)

@return cv::Point A vector containing the desired angle for each axis
*/
cv::Point DualAxisController::AngleControl(int desPos_x, int desPos_y,
                                           int curPos_x, int curPos_y)
{
    setXYDesiredPosition_px(desPos_x, desPos_y);

    return AngleControl(curPos_x, curPos_y);
}

/**
Allows current positions to be passed in as ints. Wraps AngleControl(cv::Point)

@param curPos_x The current X position (in pixels)
@param curPos_y The current Y position (in pixels)

@return cv::Point A vector containing the desired angle for each axis
*/
cv::Point DualAxisController::AngleControl(int curPos_x, int curPos_y)
{
    cv::Point xy = cv::Point(curPos_x, curPos_y);

    return AngleControl(xy);
}

/**
Controls the angle of the X and Y axis. Only one axis will be tilted from the
zero position at a time.

@param xyCurPos_px The current position of the ball in (x, y) (in pixels)

@return cv::Point A vector containing the desired angle for each axis
*/
cv::Point DualAxisController::AngleControl(cv::Point xyCurPos_px)
{
    //NOTE: AngleControl is called once per frame
    this->xyCurPos_px = xyCurPos_px;
    ComputeError();

    int xTiltAngle = outputZero;
    int yTiltAngle = outputZero;

    /* Like the single axis tilt controller, we want to tilt and wait for
    specified periods. However, we also only want to control a single axis
    at a time. */

    // If we're already controlling X, keep controlling it. 
    if (controllingX)
    {
        // If X is within the minimum position error, stop controlling it,
        // reset the X timers and do nothing else this frame.
        if (abs(error.x) <= minimumPositionError)
        {
            controllingX = false;
            this->tiltStartTick = 0;
        }
        else
        {
            // We're controlling X, it's outside of the minimum position error,
            // let's move it.
            xTiltAngle = GetTilt();
        }
    }
    // else if we're controlling Y, keep controlling it.
    else if (controllingY) {
        // If Y is within min pos, stop controlling and reset Y timers.
        if (abs(error.y) <= minimumPositionError)
        {
            controllingY = false;
            this->tiltStartTick = 0;
        }
        else
        {
            // Move Y!
            yTiltAngle = GetTilt();
        }

    }
    // else if we're controlling neither, see which one (if any) is above the
    // minimum position error, and start controlling it.
    else if (abs(error.x) > minimumPositionError)
    {
        controllingX = true;
        xTiltAngle = GetTilt();
    }
    else if (abs(error.y) > minimumPositionError)
    {
        controllingY = true;
        yTiltAngle = GetTilt();
    }
    // else, we're at our desired position within +- the minimum error

    xTiltAngle = NormalizeAngle(xTiltAngle);
    yTiltAngle = NormalizeAngle(yTiltAngle);

    return cv::Point(xTiltAngle, yTiltAngle);
}

int DualAxisController::GetTilt()
{
    double currentTick = (double)cv::getTickCount();
    double diff = (currentTick - tiltStartTick) * 1000 / cv::getTickFrequency();

    /* Diff is now the time elapsed in milliseconds since the start of the last
    tilt */

    int tiltAngle = 0;
    // If diff <= tiltActionTime, we are still inside a tilt action. So we tilt!
    if (diff <= tiltActionTime)
    {
        if (controllingX)
            tiltAngle = DetermineXTiltAngle();
        else
            tiltAngle = DetermineYTiltAngle();
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
        if (controllingX)
            tiltAngle = DetermineXTiltAngle();
        else
            tiltAngle = DetermineYTiltAngle();

        // Update the tilt start tick to the current tick so we're back to a 0
        // diff for future calculations
        tiltStartTick = currentTick;
    }

    return tiltAngle;
}

int DualAxisController::DetermineXTiltAngle()
{
    if (error.x > 0)
        return outputMin_deg;
    else
        return outputMax_deg;
}

int DualAxisController::DetermineYTiltAngle()
{
    if (error.y > 0)
        return outputMax_deg;
    else
        return outputMin_deg;
}

int DualAxisController::NormalizeAngle(int angle)
{
    return (int)((angle - LOWERLIMIT) * \
        (outputMax_deg - outputMin_deg) / (UPPERLIMIT - LOWERLIMIT));
}

bool DualAxisController::AtDesiredPosition()
{
    return abs(error.x) <= minimumPositionError && \
        abs(error.y) <= minimumPositionError;
}

/*************************
********* Setters ********
*************************/
void DualAxisController::setXYCurrentPosition_px(int curPos_x, int curPos_y)
{
    setCurrentPosition_px(cv::Point(curPos_x, curPos_y));
}

void DualAxisController::setCurrentPosition_px(cv::Point xyCurPos_px)
{
    this->xyCurPos_px = xyCurPos_px;
}

void DualAxisController::setXYDesiredPosition_px(int desPos_x, int desPos_y)
{
    setDesiredPosition_px(cv::Point(desPos_x, desPos_y));
}

void DualAxisController::setDesiredPosition_px(cv::Point xyDesPos_px)
{
    this->xyDesPos_px = xyDesPos_px;
}

void DualAxisController::setMinimumPositionError(double minimumPositionError)
{
    this->minimumPositionError = minimumPositionError;
}

void DualAxisController::setTiltDelay(double tiltDelay)
{
    this->tiltDelay = tiltDelay;
}

void DualAxisController::setTiltActionTime(double tiltActionTime)
{
    this->tiltActionTime = tiltActionTime;
}

/*************************
********* Getters ********
*************************/
double DualAxisController::getMinumumPositionError()
{
    return this->minimumPositionError;
}

double DualAxisController::getTiltDelay()
{
    return this->tiltDelay;
}

double DualAxisController::getTiltActionTime()
{
    return this->tiltActionTime;
}

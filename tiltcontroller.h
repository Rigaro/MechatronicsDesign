#ifndef CONTROLLER_TILT_H
#define CONTROLLER_TILT_H

#include "controller.h"

class TiltController : public Controller
{
    public:
        TiltController();
        TiltController(int, int, int, double, double, double);

        int PositionControl(int);
        int PositionControl(int, int);
        int PositionControl(int, int, double);

        void setMinimumPositionError(double);
        double getMinumumPositionError();

        void setTiltDelay(double tiltDelay);
        double getTiltDelay();

        void setTiltActionTime(double tiltActionTime);
        double getTiltActionTime();

    private:
        double tiltDelay;
        double tiltActionTime;
        double tiltStartTick;

        int outputZero;

        double minimumPositionError;

        void initialize();
        int GetTilt();
        int DetermineTiltAngle();
};

#endif

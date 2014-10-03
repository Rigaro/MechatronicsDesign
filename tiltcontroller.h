#ifndef CONTROLLER_TILT_H
#define CONTROLLER_TILT_H

#include "controller.h"

class TiltController : public Controller
{
    public:
        TiltController();
        TiltController(int, int, int, double, double, double);

        int positionControl(int);
        int positionControl(int, int);
        int positionControl(int, int, double);

        void setTiltDelay(double tiltDelay);
        double getTiltDelay();

        void setTiltActionTime(double tiltActionTime);
        double getTiltActionTime();

    private:
        double tiltDelay;
        double tiltActionTime;
        double tiltStartTick;

        int outputZero;



        void initialize();
        int GetTilt();
        int DetermineTiltAngle();
};

#endif

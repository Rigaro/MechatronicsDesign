#ifndef CONTROLLER_TILT_H
#define CONTROLLER_TILT_H

#include "controller.h"

class TiltController : public Controller
{
    public:
        TiltController();
        TiltController(int, int, double, double, double);

        int PositionControl(int, int);
        int PositionControl(int, int, double);

        void setMinimumPositionError(double);
        double getMinumumPositionError();

    private:
        double tiltDelay;
        double tiltActionTime;
        double minimumPositionError;

        void initialize();
};

#endif

#ifndef CONTROLLER_TILT_H
#define CONTROLLER_TILT_H

#include "controller.h"

class ControllerTilt : public Controller
{
    public:
        ControllerTilt();
        ControllerTilt(int, int, double);

        int PositionControl(int, int);

    private:
        double tiltDelay;

        void initialize();
};

#endif
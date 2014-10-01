#ifndef CONTROLLERPID_H
#define CONTROLLERPID_H

#include "controller.h"
class ControllerPID : public Controller
{
    public:
        ControllerPID();
        ControllerPID(int, int, double, double, double);
        int PositionControl(int, int); // calculate frame delta w/ sample freq
        int PositionControl(int, int, double); // provide frame delta & des pos
        int PositionControl(int, double); // provide frame delta and curr pos
        void setGainI(double);
        void setGainD(double);
        double getGainI();
        double getGainD();

    private:
        double gainI;
        double gainD;
        double integral;
        double derivative;
        double samplingFreq;
        double previousError;

        void ComputeError();
        double IntegralCorrection(double delta);
        double DerivativeCorrection(double delta);
};

#endif // CONTROLLERPID_H

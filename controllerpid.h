#ifndef CONTROLLERPID_H
#define CONTROLLERPID_H

#include "controller.h"
class ControllerPID : public Controller
{
    public:
        ControllerPID();
        ControllerPID(int, int, double, double, double);
        int positionControl(int, int); // calculate frame delta w/ sample freq
        int positionControl(int, int, double); // provide frame delta & des pos
        int positionControl(int, double); // provide frame delta and curr pos
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

        void computeError();
        double integralCorrection(double delta);
        double derivativeCorrection(double delta);
};

#endif // CONTROLLERPID_H

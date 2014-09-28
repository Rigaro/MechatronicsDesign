#ifndef CONTROLLERPID_H
#define CONTROLLERPID_H

#include "controller.h"

class ControllerPID : public Controller
{
public:
    ControllerPID();
    ControllerPID(int, int, double, double, double, double);
    int PositionControl(int, int);
    int PositionControl(int);
private:
    double gainI;
    double gainD;
    double integral;
    double derivative;
    double samplingFreq;
    double IntegralCorrection();
    double DerivativeCorrection();
};

#endif // CONTROLLERPID_H

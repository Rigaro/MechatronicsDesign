#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "controller.h"

class FSController: public Controller
{
    public:
        FSController();
        FSController(int, int, double, double);

        int positionControl(int, int, double);
        int positionControl(int, double);
    
    protected:
        double observeDerivativeState();
        int ConvertMMToPixel(double mm);
        double ConvertPixelToMM(int pixel);

    private:
        double state1Gain;
        double state2Gain;

        double previousObservedStateValue;

        void initialize();
};




#endif // FSCONTROLLER_H

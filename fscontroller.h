#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "controller.h"

class FSController: public Controller
{
public:
    FSController();
    
    FSController(int, int, int, int);
    
private:
    double gainD;
};




#endif // FSCONTROLLER_H

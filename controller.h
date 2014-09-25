#ifndef CONTROLLER_H
#define CONTROLLER_H

#define UPPERLIMIT 100
#define LOWERLIMIT -100

class Controller
{
public:
    Controller();
    Controller(int, int, double);
    void SetOutputMax(int);
    void SetOutputMin(int);
    void SetGainP(double);
    void SetDesiredPos(int);
    void SetCurrentPos(int);
    int GetOutputMax();
    int GetOutputMin();
    double GetGainP();
    int GetCurrentPos();
    int GetDesiredPos();
    int GetErrorNew();
    int PositionControl(int, int);
    int PositionControl(int);
private:
    int errorNew;
    int errorPast;
    int desiredPos;
    int currentPos;
    double output;
    int outputMin;
    int outputMax;
    double gainP;
    void ComputeError();
    double ProportionalCorrection();
    int NormalizeData(int);
};

#endif // CONTROLLER_H

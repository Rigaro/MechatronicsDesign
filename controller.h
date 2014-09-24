#ifndef CONTROLLER_H
#define CONTROLLER_H

#define UPPERLIMIT 100
#define LOWERLIMIT -100

class Controller
{
public:
    Controller();
    Controller(int, int, int);
    void SetOutputMax(int);
    void SetOutputMin(int);
    void SetGainP(int);
    void SetDesiredPos(int);
    void SetCurrentPos(int);
    int GetOutputMax();
    int GetOutputMin();
    int GetGainP();
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
    int output;
    int outputMin;
    int outputMax;
    int gainP;
    void ComputeError();
    int ProportionalCorrection();
    int NormalizeData(int);
};

#endif // CONTROLLER_H

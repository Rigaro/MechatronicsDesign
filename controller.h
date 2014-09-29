#ifndef CONTROLLER_H
#define CONTROLLER_H

#define UPPERLIMIT 100
#define LOWERLIMIT -100

class Controller
{
    public:
        Controller();
        Controller(int, int, double);
        int PositionControl(int, int);
        int PositionControl(int);
        void SetDesiredPos_px(int);
        void SetDesiredPos_mm(double);
        void SetCurrentPos_px(int);
        void SetCurrentPos_mm(double);
        void SetGainP(double);
        void SetOutputMax(int);
        void SetOutputMin(int);
        double GetGainP();
        int GetOutputMax();
        int GetOutputMin();
        int GetDesiredPos_px();
        double GetDesiredPos_mm();
        int GetCurrentPos_px();
        double GetCurrentPos_mm();
        int GetCurrentError();
    protected:
        int error;
        double controlSignal;
    
        int desPos_px;
        double desPos_mm;
        int curPos_px;
        double curPos_mm;

        void ComputeError();
        double ProportionalCorrection();
        int NormalizeData(int);
        int ClampSaturation(int);
    private:
        double gainP;
        int outputMin_deg;
        int outputMax_deg;
};

#endif // CONTROLLER_H

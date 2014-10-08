#ifndef CONTROLLER_H
#define CONTROLLER_H

#define UPPERLIMIT 1000
#define LOWERLIMIT -1000

#define BALL_RAD 12

class Controller
{
    public:
        Controller();
        Controller(int, int, double);
        int positionControl(int, int);
        int positionControl(int);
        void setDesiredPos_px(int);
        void setDesiredPos_mm(double);
        void setCurrentPos_px(int);
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

        bool atDesiredPosition();

        void setMinimumPositionError(double);
        double getMinumumPositionError();

    protected:
        int error;
   
        int desPos_px;
        double desPos_mm;
        int curPos_px;
        double curPos_mm;

        double minimumPositionError;


        void computeError();
        double proportionalCorrection();
        int normalizeData(double);
        double clampSaturation(double);
    private:
        double gainP;
        int outputMin_deg;
        int outputMax_deg;
};

#endif // CONTROLLER_H

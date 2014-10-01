#ifndef DUAL_AXIS_CONTROLLER_H
#define DUAL_AXIS_CONTROLLER_H

#include "controller.h"
#include "opencv2/core/core.hpp"

class DualAxisController
{
    public:
        DualAxisController();

        DualAxisController(int, int, int, double, double, double);

        cv::Point AngleControl(int, int);
        cv::Point AngleControl(int, int, int, int);
        cv::Point AngleControl(cv::Point);

        void setXYDesiredPosition_px(int, int);
        void setDesiredPosition_px(cv::Point);
        cv::Point getDesiredPosition_px();

        void setXYCurrentPosition_px(int, int);
        void setCurrentPosition_px(cv::Point);
        cv::Point getCurrentPosition_px();
        
        void setMinimumPositionError(double);
        double getMinumumPositionError();

        void setTiltDelay(double tiltDelay);
        double getTiltDelay();

        void setTiltActionTime(double tiltActionTime);
        double getTiltActionTime();

        bool AtDesiredPosition();

    protected:
        cv::Point error;

        void ComputeError();

        int GetTilt();

        int DetermineXTiltAngle();
        int DetermineYTiltAngle();

        int NormalizeAngle(int angle);

    private:
        cv::Point xyDesPos_px;
        cv::Point xyCurPos_px;

        double tiltDelay;
        double tiltActionTime;
        double tiltStartTick;

        int outputMin_deg;
        int outputMax_deg;
        int outputZero;

        bool controllingX;
        bool controllingY;

        double minimumPositionError;
};

#endif

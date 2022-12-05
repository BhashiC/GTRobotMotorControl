#pragma once
#include <Arduino.h>

class robotKinematics
{
public:
    robotKinematics(double wheelRadius, double wheelBase, double robotOmegaXMax, double robotOmegaXMin, double robotOmegaZMax, double robotOmegaZMin);
    double getRobotXdot();
    double getRobotZdot();
    double getLeftWheelOmega();
    double getRightWheelOmega();
    double getRobotXdotMax();
    double getRobotXdotMin();
    double getRobotZdotMax();
    double getRobotZdotMin();
    bool getIsRobotStop();
    void setRobotXdot(double val);
    void setRobotZdot(double val);
    void setIsRobotStop(bool val);
    void printKinematicsData();

private:
    double _leftWheelOmega;
    double _rightWheelOmega;
    double _robotXdot;
    double _robotZdot;
    double _wheelRadius;
    double _wheelBase;
    double _robotXdotMax;
    double _robotXdotMin;
    double _robotZdotMax;
    double _robotZdotMin;
    double _radPersecToRpm;
    bool _isRobotStop;
};
#pragma once

#include <Arduino.h>
#include <motorController.h>
#include <robotKinematics.h>

class ps2Handler
{
public:
    ps2Handler(motorController &leftMotorPtr, motorController &rightMotorPtr, robotKinematics &kinematicsModelPtr);
    void readPs2Data();

private:
    motorController *_leftMotorPtr;
    motorController *_rightMotorPtr;
    robotKinematics *_kinematicsModelPtr;
    double _robotSpeed;
    double _leftGearY;
    double _leftGearX;
    double _leftGearDefaultY;
    double _leftGearDefaultX;
    void _achieveRpms();
    // Right Joystick Gear is not use to avoid value conflict
};
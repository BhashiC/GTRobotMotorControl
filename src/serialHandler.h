#pragma once

#include <Arduino.h>
#include <motorController.h>
#include <robotKinematics.h>

class serialHandler
{
public:
    serialHandler(motorController &leftMotorPtr, motorController &rightMotorPtr, robotKinematics &kinematicsModelPtr);
    void readSerialData();
    void printEncoderCounts();
    void printSpeeds(int leftMotorPwm, int rightMotorPwm);

private:
    motorController *_leftMotorPtr;
    motorController *_rightMotorPtr;
    robotKinematics *_kinematicsModelPtr;
};

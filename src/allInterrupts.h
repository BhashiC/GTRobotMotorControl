#pragma once

#include <Arduino.h>
#include <motorController.h>

class allInterrupts
{
public:
    friend void leftMotorCountEnA();
    friend void leftMotorCountEnB();
    friend void rightMotorCountEnA();
    friend void rightMotorCountEnB();
    static void attachInterruptFuncs();
    static motorController *leftMotorPtr;
    static motorController *rightMotorPtr;
    
private:
};
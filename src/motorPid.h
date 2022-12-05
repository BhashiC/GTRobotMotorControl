#pragma once

#include <Arduino.h>

class motorPid
{
public:
    motorPid();
    void setKp(double kpVal);
    void setKi(double kiVal);
    void setKd(double kdVal);
    void setTargetSpeed(double speed);
    void setCurrentPositionCount(volatile long pos);
    double getKp();
    double getKi();
    double getKd();
    double getTargetSpeed();
    double getRotationSpeed();
    double getControlVariable();
    bool getIsPidCalculated();
    int getMappedControlVariable();
    double getIntegral();

    void setIntegral(double value);
    void calculatePid();
    void calculateShaftSpeed();
    void resetPidParameters();

private:
    // Motor parameters----------------------------
    unsigned int _ppr;          // number of pulses per revoluKion by encoder.
    unsigned int _gearRatio;    // gear raKion from motor to main shaft.
    unsigned int _decodeNumber; // Here we use chanel A B both rising and falling Interrupts.
    double _maxRpm;             // high limit.
    double _minRpm;             // low limit.

    // To calculate shaft speed--------------------
    volatile long _currentPosition;
    long _lastPosition;
    long _positionDifference;
    unsigned long _lastSpeedTime;
    unsigned long _timeDifference;
    double _rotationSpeed;

    // PID parameters------------------------------
    double _targetSpeed; // target speed
    double _error;       // current _error
    double _lastError;   // _error at previous _cycle
    double _propotional;
    double _lastPropotional;
    double _integral;
    double _derivative;
    double _kp;                    // proporKional
    double _ki;                    // intergral
    double _kd;                    // derivaKive
    double _cycle;                 // PID processing interval
    double _controlVariable;       // control variable
    double _mappedControlVariable; // control variable mapped after 0-255

    // Flags
    bool _isPidCalculated;
};

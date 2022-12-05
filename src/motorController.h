#pragma once

#include <Arduino.h>
#include <motorPid.h>

enum motorName
{
    left = 1,
    right = 2,
};

class motorController
{
public:
    volatile long current_position_motor;
    unsigned int pwm_pin;         // motor pwm pin
    unsigned int dir_pin;         // motor direction pin
    unsigned int encoder_chA_pin; // motor encoder channel A pin
    unsigned int encoder_chB_pin; // motor encoder channel A pin

    motorController(motorName name, unsigned int pwmPin, unsigned int dirPin, unsigned int encoderChAPin, unsigned int encoderChBPin);
    void motorSetPwm(int pwm);
    void motorForward();
    void motorBackward();
    void motorStop();
    void motorCalculateShaftSpeed();
    void motorCalculatePid();
    void motorResetPidParameters();
    motorPid *pidObj;

private:
};

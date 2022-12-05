#include <motorController.h>

/// @brief Constructor
/// @param pwmPin motor pwm pin
/// @param dirPin motor direction pin
/// @param encoderChAPin motor encoder channel A pin
/// @param encoderChaBPin motor encoder channel A pin
motorController::motorController(motorName name, unsigned int pwmPin, unsigned int dirPin, unsigned int encoderChAPin, unsigned int encoderChBPin)
{
    pidObj = new motorPid();

    pwm_pin = pwmPin;
    dir_pin = dirPin;
    encoder_chA_pin = encoderChAPin; // encoderChAPin;
    encoder_chB_pin = encoderChBPin; // encoderChBPin;

    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(pwm_pin, LOW);
    digitalWrite(dir_pin, LOW);

    pinMode(encoder_chA_pin, INPUT);
    pinMode(encoder_chB_pin, INPUT);
}

/// @brief Set motor pwm value
/// @param pwm pwm value
void motorController::motorSetPwm(int pwm)
{
    if (pwm < 0 || pwm > 255)
    {
        Serial.println("Wrong Input: PWM must be 0-255");
        return;
    }
    analogWrite(pwm_pin, pwm);
}

/// @brief Set motor direction forward
void motorController::motorForward()
{
    digitalWrite(dir_pin, HIGH);
}

/// @brief Set motor direction backward
void motorController::motorBackward()
{
    digitalWrite(dir_pin, LOW);
}

/// @brief Stop motor
void motorController::motorStop()
{
    analogWrite(pwm_pin, 0);
    motorResetPidParameters();
}

/// @brief Calculate motor output shaft speed
void motorController::motorCalculateShaftSpeed()
{
    pidObj->calculateShaftSpeed();
}

/// @brief Calculate pid for the motor
void motorController::motorCalculatePid()
{
    pidObj->calculatePid();
    motorSetPwm(pidObj->getMappedControlVariable());
}

/// @brief Reset pid parameters for the motor
void motorController::motorResetPidParameters()
{
    pidObj->resetPidParameters();
}
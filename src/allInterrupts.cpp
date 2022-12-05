#include <allInterrupts.h>

/// @brief Left motor encoder channel A handler function
void leftMotorCountEnA()
{
    if (digitalRead(allInterrupts::leftMotorPtr->encoder_chA_pin) != digitalRead(allInterrupts::leftMotorPtr->encoder_chB_pin))
    {
        allInterrupts::leftMotorPtr->current_position_motor++;
    }
    else
    {
        allInterrupts::leftMotorPtr->current_position_motor--;
    }
}

/// @brief Left motor encoder channel B handler function
void leftMotorCountEnB()
{
    if (digitalRead(allInterrupts::leftMotorPtr->encoder_chA_pin) == digitalRead(allInterrupts::leftMotorPtr->encoder_chB_pin))
    {
        allInterrupts::leftMotorPtr->current_position_motor++;
    }
    else
    {
        allInterrupts::leftMotorPtr->current_position_motor--;
    }
}

/// @brief Right motor encoder channel A handler function
void rightMotorCountEnA()
{
    if (digitalRead(allInterrupts::rightMotorPtr->encoder_chA_pin) != digitalRead(allInterrupts::rightMotorPtr->encoder_chB_pin))
    {
        allInterrupts::rightMotorPtr->current_position_motor++;
    }
    else
    {
        allInterrupts::rightMotorPtr->current_position_motor--;
    }
}

/// @brief Right motor encoder channel B handler function
void rightMotorCountEnB()
{
    if (digitalRead(allInterrupts::rightMotorPtr->encoder_chA_pin) == digitalRead(allInterrupts::rightMotorPtr->encoder_chB_pin))
    {
        allInterrupts::rightMotorPtr->current_position_motor++;
    }
    else
    {
        allInterrupts::rightMotorPtr->current_position_motor--;
    }
}

void allInterrupts::attachInterruptFuncs()
{
    attachInterrupt(digitalPinToInterrupt(allInterrupts::leftMotorPtr->encoder_chA_pin), leftMotorCountEnA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(allInterrupts::leftMotorPtr->encoder_chB_pin), leftMotorCountEnB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(allInterrupts::rightMotorPtr->encoder_chA_pin), rightMotorCountEnA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(allInterrupts::rightMotorPtr->encoder_chB_pin), rightMotorCountEnB, CHANGE);
}

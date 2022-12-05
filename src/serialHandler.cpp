#include <serialHandler.h>

serialHandler::serialHandler(motorController &leftMotorPtr, motorController &rightMotorPtr, robotKinematics &kinematicsModelPtr)
{
    _leftMotorPtr = &leftMotorPtr;
    _rightMotorPtr = &rightMotorPtr;
    _kinematicsModelPtr = &kinematicsModelPtr;
}

/// @brief Read serial commands from the user
void serialHandler::readSerialData()
{
    if (Serial.available() > 0)
    {
        char ch = Serial.read();
        switch (ch)
        {
        case '[':
            _leftMotorPtr->motorForward();
            Serial.println("Left Motor Forward!");
            break;
        case ']':
            _rightMotorPtr->motorForward();
            Serial.println("Right Motor Forward!");
            break;
        case '<':
            _leftMotorPtr->motorBackward();
            _rightMotorPtr->motorForward();
            Serial.println("Robot Counterclockwise!");
            break;
        case '>':
            _leftMotorPtr->motorForward();
            _rightMotorPtr->motorBackward();
            Serial.println("Robot Clockwise!");
            break;
        case 'F':
            _leftMotorPtr->motorForward();
            _rightMotorPtr->motorForward();
            Serial.println("Motors Forward!");
            break;
        case 'B':
            _leftMotorPtr->motorBackward();
            _rightMotorPtr->motorBackward();
            Serial.println("Motors Backward!");
            break;
        case 'X':
            _leftMotorPtr->current_position_motor = 0;
            _rightMotorPtr->current_position_motor = 0;
            break;

        case 'S':
        {
            int pwm = Serial.parseInt();
            if (pwm == 0)
            {
                _leftMotorPtr->pidObj->setTargetSpeed(0);
                _rightMotorPtr->pidObj->setTargetSpeed(0);
                _kinematicsModelPtr->setIsRobotStop(true);
                _leftMotorPtr->motorStop();
                _rightMotorPtr->motorStop();
                Serial.println("Motors Stop!");
            }
            else if (pwm < 0 || pwm > 255)
            {
                Serial.println("Wrong Input: PWM must be 0-255");
            }
            else
            {
                _leftMotorPtr->motorSetPwm(pwm);
                _rightMotorPtr->motorSetPwm(pwm);
                Serial.print("Motors PWM = ");
                Serial.println(pwm);
            }
            break;
        }
        case 'C':
        {
            double speedVal = Serial.parseInt();
            _leftMotorPtr->pidObj->setIntegral(0);
            _rightMotorPtr->pidObj->setIntegral(0);
            if ((speedVal >= 0) && (speedVal <= 100))
            {
                _kinematicsModelPtr->setIsRobotStop(false);
                _leftMotorPtr->pidObj->setTargetSpeed(speedVal);
                _rightMotorPtr->pidObj->setTargetSpeed(speedVal);
                Serial.print("left target speed is: ");
                Serial.print(_leftMotorPtr->pidObj->getTargetSpeed());
                Serial.print(" rpm");
                Serial.print(" <<*>> Right target speed is: ");
                Serial.print(_rightMotorPtr->pidObj->getTargetSpeed());
                Serial.println(" rpm");
                Serial.println("==============================================");
            }
            else
            {
                Serial.println("Wrong setting, enter value from 0 - 100 rpm!");
                Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
            }
            break;
        }

        case 'Q':
            Serial.println("----------------");
            Serial.print("Left Kp = ");
            Serial.println(_leftMotorPtr->pidObj->getKp(), 4); // print 04 digits after decimal point.
            Serial.print("Left Ki= ");
            Serial.println(_leftMotorPtr->pidObj->getKi(), 6);
            Serial.print("Left Kd = ");
            Serial.println(_leftMotorPtr->pidObj->getKd(), 4);
            Serial.println("----------------");
            Serial.print("Right Kp = ");
            Serial.println(_rightMotorPtr->pidObj->getKp(), 4); // print 04 digits after decimal point.
            Serial.print("Right Ki= ");
            Serial.println(_rightMotorPtr->pidObj->getKi(), 6);
            Serial.print("Right Kd = ");
            Serial.println(_rightMotorPtr->pidObj->getKd(), 4);
            Serial.println("========================================================");
            break;

        case 'P':
            _leftMotorPtr->pidObj->setKp(_leftMotorPtr->pidObj->getKp() + 0.05); // += 0.005; // Kp = Kp + ...
            Serial.print("Left Motor Kp is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKp(), 4); // print 04 digits after decimal point.
            Serial.println("========================================================");
            break;

        case 'p':
            _leftMotorPtr->pidObj->setKp(_leftMotorPtr->pidObj->getKp() - 0.05); // += 0.005; // Kp = Kp + ...
            Serial.print("Left Motor Kp is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKp(), 4); // print 04 digits after decimal point.
            Serial.println("========================================================");
            break;

        case 'I':
            _leftMotorPtr->pidObj->setKi(_leftMotorPtr->pidObj->getKi() + 0.01);
            Serial.print("Left Motor Ki is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;

        case 'i':
            _leftMotorPtr->pidObj->setKi(_leftMotorPtr->pidObj->getKi() - 0.01);
            Serial.print("Left Motor Ki is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;

        case 'D':
            _leftMotorPtr->pidObj->setKd(_leftMotorPtr->pidObj->getKd() + 0.01);
            Serial.print("Left Motor Kd is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKd());
            Serial.println("========================================================");
            break;

        case 'd':
            _leftMotorPtr->pidObj->setKd(_leftMotorPtr->pidObj->getKd() - 0.01);
            Serial.print("Left Motor Kd is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKd());
            Serial.println("========================================================");
            break;

        case 'J':
            _rightMotorPtr->pidObj->setKp(_rightMotorPtr->pidObj->getKp() + 0.05); // += 0.005; // Kp = Kp + ...
            Serial.print("Right Motor Kp is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKp(), 4); // print 04 digits after decimal point.
            Serial.println("========================================================");
            break;

        case 'j':
            _rightMotorPtr->pidObj->setKp(_rightMotorPtr->pidObj->getKp() - 0.05); // += 0.005; // Kp = Kp + ...
            Serial.print("Right Motor Kp is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKp(), 4); // print 04 digits after decimal point.
            Serial.println("========================================================");
            break;

        case 'K':
            _rightMotorPtr->pidObj->setKi(_rightMotorPtr->pidObj->getKi() + 0.01);
            Serial.print("Right Motor Ki is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;

        case 'k':
            _rightMotorPtr->pidObj->setKi(_rightMotorPtr->pidObj->getKi() - 0.01);
            Serial.print("Right Motor Ki is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;

        case 'L':
            _rightMotorPtr->pidObj->setKd(_rightMotorPtr->pidObj->getKd() + 0.01);
            Serial.print("Right Motor Kd is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKd());
            Serial.println("========================================================");
            break;

        case 'l':
            _rightMotorPtr->pidObj->setKd(_rightMotorPtr->pidObj->getKd() - 0.01);
            Serial.print("Right Motor Kd is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKd());
            Serial.println("========================================================");
            break;
        }
    }
}

/// @brief print left and right motors' speeds
/// @param leftMotorPwm left motor pwm output
/// @param rightMotorPwm right motor pwm output
void serialHandler::printSpeeds(int leftMotorPwm, int rightMotorPwm)
{
    Serial.print("left_target_speed = ");
    Serial.print(_leftMotorPtr->pidObj->getTargetSpeed(), 2);
    Serial.print("  left_current_speed = ");
    Serial.print(_leftMotorPtr->pidObj->getRotationSpeed(), 2);
    Serial.print("  pwm = ");
    Serial.print(leftMotorPwm);
    Serial.print("  integral = ");
    Serial.print(_leftMotorPtr->pidObj->getIntegral());
    Serial.print(" <<*>>> ");
    Serial.print("right_target_speed = ");
    Serial.print(_rightMotorPtr->pidObj->getTargetSpeed(), 2);
    Serial.print("  right_current_speed = ");
    Serial.print(_rightMotorPtr->pidObj->getRotationSpeed(), 2);
    Serial.print("  pwm = ");
    Serial.print(rightMotorPwm);
    Serial.print("  integral = ");
    Serial.println(_rightMotorPtr->pidObj->getIntegral());
}

/// @brief print left and right motors' encoder counts
void serialHandler::printEncoderCounts()
{
    Serial.print("current_position_motor_left = ");
    Serial.print(_leftMotorPtr->current_position_motor);
    Serial.print("  current_position_motor_right = ");
    Serial.println(_rightMotorPtr->current_position_motor);
}

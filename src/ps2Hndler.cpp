#include <ps2Handler.h>

ps2Handler::ps2Handler(motorController &leftMotorPtr, motorController &rightMotorPtr, robotKinematics &kinematicsModelPtr)
{
    _leftMotorPtr = &leftMotorPtr;
    _rightMotorPtr = &rightMotorPtr;
    _kinematicsModelPtr = &kinematicsModelPtr;

    _robotSpeed = 30; // rpm
    _leftGearY = 0;
    _leftGearX = 0;
    _leftGearDefaultY = 127;
    _leftGearDefaultX = 128;
}

/// @brief Read ps2 commands from the user
void ps2Handler::readPs2Data()
{
    if (Serial2.available() > 0)
    {
        char ch = Serial2.read();
        switch (ch)
        {
        case 'I':
            _leftMotorPtr->pidObj->setIntegral(0);
            _rightMotorPtr->pidObj->setIntegral(0);
            _kinematicsModelPtr->setRobotXdot(0);
            _kinematicsModelPtr->setRobotZdot(0);
            _leftMotorPtr->motorForward();
            _rightMotorPtr->motorForward();
            _leftMotorPtr->pidObj->setTargetSpeed(30);
            _rightMotorPtr->pidObj->setTargetSpeed(30);
            _kinematicsModelPtr->setIsRobotStop(false);
            Serial.println("Robot Forward with 50 rmp");
            break;
        case 'J':
            _leftMotorPtr->pidObj->setIntegral(0);
            _rightMotorPtr->pidObj->setIntegral(0);
            _kinematicsModelPtr->setRobotXdot(0);
            _kinematicsModelPtr->setRobotZdot(0);
            _leftMotorPtr->motorBackward();
            _rightMotorPtr->motorBackward();
            _leftMotorPtr->pidObj->setTargetSpeed(30);
            _rightMotorPtr->pidObj->setTargetSpeed(30);
            _kinematicsModelPtr->setIsRobotStop(false);
            Serial.println("Robot Backward!");
            break;
        case 'K':
            _leftMotorPtr->pidObj->setIntegral(0);
            _rightMotorPtr->pidObj->setIntegral(0);
            _kinematicsModelPtr->setRobotXdot(0);
            _kinematicsModelPtr->setRobotZdot(0);
            _leftMotorPtr->motorBackward();
            _rightMotorPtr->motorForward();
            _leftMotorPtr->pidObj->setTargetSpeed(30);
            _rightMotorPtr->pidObj->setTargetSpeed(30);
            _kinematicsModelPtr->setIsRobotStop(false);
            Serial.println("Robot Left Rotate!");
            break;
        case 'L':
            _leftMotorPtr->pidObj->setIntegral(0);
            _rightMotorPtr->pidObj->setIntegral(0);
            _kinematicsModelPtr->setRobotXdot(0);
            _kinematicsModelPtr->setRobotZdot(0);
            _leftMotorPtr->motorForward();
            _rightMotorPtr->motorBackward();
            _leftMotorPtr->pidObj->setTargetSpeed(30);
            _rightMotorPtr->pidObj->setTargetSpeed(30);
            _kinematicsModelPtr->setIsRobotStop(false);
            Serial.println("Robot Right Rotate!");
            break;
        case 'G':
        {
            _leftMotorPtr->pidObj->setIntegral(0);
            _rightMotorPtr->pidObj->setIntegral(0);
            _kinematicsModelPtr->setRobotXdot(0);
            _kinematicsModelPtr->setRobotZdot(0);
            _leftMotorPtr->pidObj->setTargetSpeed(0);
            _rightMotorPtr->pidObj->setTargetSpeed(0);
            _leftMotorPtr->motorStop();
            _rightMotorPtr->motorStop();
            _kinematicsModelPtr->setIsRobotStop(true);
            Serial.println("Robot Stop!");
            break;
        }
        case 'A':
        {
            _kinematicsModelPtr->setRobotXdot(_kinematicsModelPtr->getRobotXdot() + 0.01);
            if (_kinematicsModelPtr->getRobotXdot() > _kinematicsModelPtr->getRobotXdotMax())
            {
                _kinematicsModelPtr->setRobotXdot(_kinematicsModelPtr->getRobotXdotMax());
            }
            if (_kinematicsModelPtr->getRobotXdot() < _kinematicsModelPtr->getRobotXdotMin())
            {
                _kinematicsModelPtr->setRobotXdot(_kinematicsModelPtr->getRobotXdotMin());
            }
            _kinematicsModelPtr->setIsRobotStop(false);
            _achieveRpms();
            /*
            _robotSpeed = _robotSpeed + 0.5;
            if (_robotSpeed >= 90)
            {
                _robotSpeed = 90;
            }
            //_leftMotorPtr->pidObj->setIntegral(0);
            //_rightMotorPtr->pidObj->setIntegral(0);
            _leftMotorPtr->pidObj->setTargetSpeed(_robotSpeed);
            _rightMotorPtr->pidObj->setTargetSpeed(_robotSpeed);
            Serial.print("left target speed is: ");
            Serial.print(_leftMotorPtr->pidObj->getTargetSpeed());
            Serial.print(" rpm");
            Serial.print(" <<*>> Right target speed is: ");
            Serial.print(_rightMotorPtr->pidObj->getTargetSpeed());
            Serial.println(" rpm");
            Serial.println("==============================================");
            */
            break;
        }
        case 'B':
        {
            _kinematicsModelPtr->setRobotXdot(_kinematicsModelPtr->getRobotXdot() - 0.01);
            if (_kinematicsModelPtr->getRobotXdot() > _kinematicsModelPtr->getRobotXdotMax())
            {
                _kinematicsModelPtr->setRobotXdot(_kinematicsModelPtr->getRobotXdotMax());
            }
            if (_kinematicsModelPtr->getRobotXdot() < _kinematicsModelPtr->getRobotXdotMin())
            {
                _kinematicsModelPtr->setRobotXdot(_kinematicsModelPtr->getRobotXdotMin());
            }
            _kinematicsModelPtr->setIsRobotStop(false);
            _achieveRpms();
            /*
            _robotSpeed = _robotSpeed - 0.5;
            if (_robotSpeed <= 0)
            {
                _robotSpeed = 0;
            }
            //_leftMotorPtr->pidObj->setIntegral(0);
            //_rightMotorPtr->pidObj->setIntegral(0);
            _leftMotorPtr->pidObj->setTargetSpeed(_robotSpeed);
            _rightMotorPtr->pidObj->setTargetSpeed(_robotSpeed);
            Serial.print("left target speed is: ");
            Serial.print(_leftMotorPtr->pidObj->getTargetSpeed());
            Serial.print(" rpm");
            Serial.print(" <<*>> Right target speed is: ");
            Serial.print(_rightMotorPtr->pidObj->getTargetSpeed());
            Serial.println(" rpm");
            Serial.println("==============================================");
            */
            break;
        }
        case 'C':
        {
            _kinematicsModelPtr->setRobotZdot(_kinematicsModelPtr->getRobotZdot() - 0.015);
            if (_kinematicsModelPtr->getRobotZdot() > _kinematicsModelPtr->getRobotZdotMax())
            {
                _kinematicsModelPtr->setRobotZdot(_kinematicsModelPtr->getRobotZdotMax());
            }
            if (_kinematicsModelPtr->getRobotZdot() < _kinematicsModelPtr->getRobotZdotMin())
            {
                _kinematicsModelPtr->setRobotZdot(_kinematicsModelPtr->getRobotZdotMin());
            }
            _kinematicsModelPtr->setIsRobotStop(false);
            _achieveRpms();
            break;
        }
        case 'D':
        {
            _kinematicsModelPtr->setRobotZdot(_kinematicsModelPtr->getRobotZdot() + 0.015);
            if (_kinematicsModelPtr->getRobotZdot() > _kinematicsModelPtr->getRobotZdotMax())
            {
                _kinematicsModelPtr->setRobotZdot(_kinematicsModelPtr->getRobotZdotMax());
            }
            if (_kinematicsModelPtr->getRobotZdot() < _kinematicsModelPtr->getRobotZdotMin())
            {
                _kinematicsModelPtr->setRobotZdot(_kinematicsModelPtr->getRobotZdotMin());
            }
            _kinematicsModelPtr->setIsRobotStop(false);
            _achieveRpms();
            break;
        }
        /*
        case 'W':
        {
            _leftGearY = Serial2.parseInt();
            if (_leftGearY > 255)
            {
                _leftGearY = 255;
            }
            else if (_leftGearY < 0)
            {
                _leftGearY = 0;
            }
            _leftGearY = _leftGearY - _leftGearDefaultY;
            Serial.print("leftGearY: ");
            Serial.print(_leftGearY);
            Serial.print(" <<*>> ");
            Serial.print("leftGearX: ");
            Serial.println(_leftGearX);
            break;
        }
        */
        /*
        case 'P':
        {
            _leftGearX = Serial2.parseInt();
            if (_leftGearX > 255)
            {
                _leftGearX = 255;
            }
            else if (_leftGearX < 0)
            {
                _leftGearX = 0;
            }
            _leftGearX = _leftGearX - _leftGearDefaultX;
            Serial.print("leftGearY: ");
            Serial.print(_leftGearY);
            Serial.print(" <<*>> ");
            Serial.print("leftGearX: ");
            Serial.println(_leftGearX);
            break;
        }
        */
        /*
        case 'H':
            _leftMotorPtr->current_position_motor = 0;
            _rightMotorPtr->current_position_motor = 0;
            break;
        */
        case 'H':
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
        case 'E':
            _leftMotorPtr->pidObj->setKi(_leftMotorPtr->pidObj->getKi() + 0.01);
            Serial.print("Left Motor Ki is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;
        case 'F':
            _leftMotorPtr->pidObj->setKi(_leftMotorPtr->pidObj->getKi() - 0.01);
            Serial.print("Left Motor Ki is changed to: ");
            Serial.println(_leftMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;
        case 'M':
            _rightMotorPtr->pidObj->setKi(_rightMotorPtr->pidObj->getKi() + 0.01);
            Serial.print("Right Motor Ki is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;
        case 'N':
            _rightMotorPtr->pidObj->setKi(_rightMotorPtr->pidObj->getKi() - 0.01);
            Serial.print("Right Motor Ki is changed to: ");
            Serial.println(_rightMotorPtr->pidObj->getKi(), 6);
            Serial.println("========================================================");
            break;
        }
    }
}

/// @brief Set motor directions and pass target Rmps to Pid loop
void ps2Handler::_achieveRpms()
{
    if (_kinematicsModelPtr->getLeftWheelOmega() >= 0)
    {
        _leftMotorPtr->motorForward();
        _leftMotorPtr->pidObj->setTargetSpeed(_kinematicsModelPtr->getLeftWheelOmega());
    }
    else
    {
        _leftMotorPtr->motorBackward();
        _leftMotorPtr->pidObj->setTargetSpeed(-_kinematicsModelPtr->getLeftWheelOmega());
    }
    if (_kinematicsModelPtr->getRightWheelOmega() >= 0)
    {
        _rightMotorPtr->motorForward();
        _rightMotorPtr->pidObj->setTargetSpeed(_kinematicsModelPtr->getRightWheelOmega());
    }
    else
    {
        _rightMotorPtr->motorBackward();
        _rightMotorPtr->pidObj->setTargetSpeed(-_kinematicsModelPtr->getRightWheelOmega());
    }
}

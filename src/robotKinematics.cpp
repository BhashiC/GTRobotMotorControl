#include <robotKinematics.h>

robotKinematics::robotKinematics(double wheelRadius, double wheelBase, double robotOmegaXMax, double robotOmegaXMin, double robotOmegaZMax, double robotOmegaZMin)
{
    _wheelRadius = wheelRadius; // in meters
    _wheelBase = wheelBase;     // in meters
    _robotXdotMax = robotOmegaXMax;
    _robotXdotMin = robotOmegaXMin;
    _robotZdotMax = robotOmegaZMax;
    _robotZdotMin = robotOmegaZMin;
    _radPersecToRpm = 9.549297;
    _isRobotStop = true;
    _robotXdot = 0;
    _robotZdot = 0;
}

/// @brief get right wheel's target velocity
/// @return value in rpm
double robotKinematics::getRightWheelOmega()
{
    return _rightWheelOmega = _radPersecToRpm * (1.00 / _wheelRadius) * (_robotXdot - _robotZdot * _wheelBase);
}

/// @brief get left wheel's target velocity
/// @return value in rpm
double robotKinematics::getLeftWheelOmega()
{
    return _leftWheelOmega = _radPersecToRpm * (1.00 / _wheelRadius) * (_robotXdot + _robotZdot * _wheelBase);
}

/// @brief get robot's Xdot vector value
/// @return value in ms-1
double robotKinematics::getRobotXdot()
{
    return _robotXdot;
}

/// @brief get robot's Zdot vector value
/// @return value in rads-1
double robotKinematics::getRobotZdot()
{
    return _robotZdot;
}

/// @brief get robot's Xdot vector maximum value
/// @return value in ms-1
double robotKinematics::getRobotXdotMax()
{
    return _robotXdotMax;
}

/// @brief get robot's Xdot vector minimum value
/// @return value in ms-1
double robotKinematics::getRobotXdotMin()
{
    return _robotXdotMin;
}

/// @brief get robot's Zot vector maximum value
/// @return value in rads-1
double robotKinematics::getRobotZdotMax()
{
    return _robotZdotMax;
}

/// @brief get robot's Zot vector minimum value
/// @return value in rads-1
double robotKinematics::getRobotZdotMin()
{
    return _robotZdotMin;
}

/// @brief get isRobotStop flag status
/// @return status
bool robotKinematics::getIsRobotStop()
{
    return _isRobotStop;
}

/// @brief set robot's Xdot vector component
/// @param val value in ms-1
void robotKinematics::setRobotXdot(double val)
{
    _robotXdot = val;
}

/// @brief set robot's Zdot vector component
/// @param val value in rads-1
void robotKinematics::setRobotZdot(double val)
{
    _robotZdot = val;
}

/// @brief set isRobotStop flag
/// @param val value
void robotKinematics::setIsRobotStop(bool val)
{
    _isRobotStop = val;
}

/// @brief prints kinematic data to serial monitor
void robotKinematics::printKinematicsData()
{
    Serial.print("robotOmegaX = ");
    Serial.print(_robotXdot, 2);
    Serial.print("  robotOmegaZ = ");
    Serial.print(_robotZdot, 2);
    Serial.print(" <<*>>> ");
    Serial.print("  leftWheelTargetRpm = ");
    Serial.print(getLeftWheelOmega(), 2);
    Serial.print("  rightWheelTargetRpm = ");
    Serial.println(getRightWheelOmega(), 2);
}

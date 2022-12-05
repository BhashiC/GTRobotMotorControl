#include <motorPid.h>

motorPid::motorPid()
{
    // PID parameters only set at the begining
    // For Floor _kp = 3.5
    _kp = 3.5;  // 0.7 // 1.2;  // 0.35; // 3.5;  // 0.95; // proporKional
    _ki = 0.08; // 0.08; //0.02// intergral
    _kd = 0.01; // derivaKive

    // Motor parameters----------------------------
    _ppr = 500;
    _gearRatio = 30;
    _decodeNumber = 4;
    _maxRpm = 100;
    _minRpm = 0;

    // PID parameters------------------------------
    _cycle = 100; // PID processing interval.
    resetPidParameters();
}

void motorPid::setKp(double kpVal)
{
    _kp = kpVal;
}

void motorPid::setKi(double kiVal)
{
    _ki = kiVal;
}

void motorPid::setKd(double kdVal)
{
    _kd = kdVal;
}

void motorPid::setTargetSpeed(double speed)
{
    _targetSpeed = speed;
}

void motorPid::setCurrentPositionCount(volatile long pos)
{
    _currentPosition = pos;
}

double motorPid::getKp()
{
    return _kp;
}

double motorPid::getKi()
{
    return _ki;
}

double motorPid::getKd()
{
    return _kd;
}

double motorPid::getTargetSpeed()
{
    return _targetSpeed;
}

double motorPid::getRotationSpeed()
{
    return _rotationSpeed;
}

double motorPid::getControlVariable()
{
    return _controlVariable;
}

int motorPid::getMappedControlVariable()
{
    return _mappedControlVariable;
}

bool motorPid::getIsPidCalculated()
{
    return _isPidCalculated;
}

double motorPid::getIntegral()
{
    return _integral;
}

void motorPid::setIntegral(double value)
{
    _integral = value;
}

/// @brief Calculate output shaft speed
void motorPid::calculateShaftSpeed()
{
    // _rotationSpeed = 60000.0*_positionDifference/(_ppr*_decodeNumber*_gearRatio*_timeDifference)
    // PPR = 500, decodeNumnber = 4, GEARraKio = 30, 60000/(500*4*30) = 1.0*/

    if (millis() - _lastSpeedTime > 50)
    {
        _timeDifference = millis() - _lastSpeedTime;
        _positionDifference = _currentPosition - _lastPosition;

        if (_positionDifference < 0)
        {
            _positionDifference = -_positionDifference;
        }

        _rotationSpeed = 1 * _positionDifference / _timeDifference;

        _lastSpeedTime = millis();
        _lastPosition = _currentPosition;
        // Serial.print("_positionDifference: ");
        // Serial.print(_positionDifference);
        // Serial.print("  _timeDifference: ");
        // Serial.print(_timeDifference);
        // Serial.print("  _rotationSpeed: ");
        // Serial.println(_rotationSpeed);
    }
}

/// @brief Calculate pid for the motor
void motorPid::calculatePid()
{
    _isPidCalculated = false;
    _error = _targetSpeed - _rotationSpeed;
    _propotional = _kp * _error;
    _integral += _ki * _propotional;
    _derivative = _kd * (_propotional - _lastPropotional);
    _lastPropotional = _propotional;
    _controlVariable = _propotional + _integral + _derivative;

    // if ((_integral > _maxRpm) && (_controlVariable > _maxRpm))
    // {
    //     _integral = 0;
    //     _controlVariable = _maxRpm;
    // }
    if ((_integral < _minRpm) && (_controlVariable < _minRpm))
    {
        _integral = 0;
        _controlVariable = _minRpm;
    }

    _mappedControlVariable = map(_controlVariable, 0, 100, 0, 255);
    if (_mappedControlVariable > 255)
    {
        _mappedControlVariable = 255;
    }
    _isPidCalculated = true;
}

/// @brief Reset pid parameters
void motorPid::resetPidParameters()
{
    // To calculate shaft speed--------------------
    _currentPosition = 0;
    _lastPosition = 0;
    _positionDifference = 0;
    _lastSpeedTime = 0;
    _timeDifference = 0;
    _rotationSpeed = 0;

    // PID parameters------------------------------
    _targetSpeed = 0; //
    _error = 0;       // current _error
    _lastError = 0;   // _error at previous _cycle
    _propotional = 0;
    _lastPropotional = 0;
    _integral = 0;
    _derivative = 0;
    _controlVariable = 0; // control variable.
    _mappedControlVariable = 0;

    // Flags
    _isPidCalculated = false;
}
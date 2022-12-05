#include <Arduino.h>
#include <motorController.h>
#include <allInterrupts.h>
#include <serialHandler.h>
#include <Scheduler.h>
#include <robotKinematics.h>
#include <ps2Handler.h>
#include <math.h>

unsigned int left_motor_pwm = 6; // pwm motor left connect to AN1 at MDS10
unsigned int left_motor_dir = 4; // direction motor left connect to DIG1 at MDS10
// Actually cnannel B is 19, but to ensure possitive encoder count when robot moving forward we changed
unsigned int left_motor_encoderA = 19; // encoder pinA motor left
// Actually cnannel A is 18, but to ensure possitive encoder count when robot moving forward we changed
unsigned int left_motor_encoderB = 18; // encoder pinB motor left

unsigned int right_motor_pwm = 5;       // pwm motor right connect to AN2 at MDS10
unsigned int right_motor_dir = 7;       // direction motor right connect to DIG2 at MDS10
unsigned int right_motor_encoderA = 20; // encoder pinB motor right
unsigned int right_motor_encoderB = 21; // encoder pinA motor right

double wheelRadius = 0.100;      // meters
double wheelBase = 0.55;         // meters
double robotOmegaXMax = 1;       // ms-1
double robotOmegaXMin = -1;      // ms-1
double robotOmegaZMax = PI / 2;  // rads-1
double robotOmegaZMin = -PI / 2; // rads-1

robotKinematics *robotKinObjPtr = new robotKinematics(wheelRadius, wheelBase, robotOmegaXMax, robotOmegaXMin, robotOmegaZMax, robotOmegaZMin);
// robotKinematics robotKinObj = *robotKinObjPtr;

motorController *allInterrupts::leftMotorPtr = new motorController(motorName::left, left_motor_pwm, left_motor_dir, left_motor_encoderA, left_motor_encoderB);
motorController *allInterrupts::rightMotorPtr = new motorController(motorName::right, right_motor_pwm, right_motor_dir, right_motor_encoderA, right_motor_encoderB);

motorController &leftMotor = *allInterrupts::leftMotorPtr;
motorController &rightMotor = *allInterrupts::rightMotorPtr;

serialHandler *serialObjPtr = new serialHandler(*allInterrupts::leftMotorPtr, *allInterrupts::rightMotorPtr, *robotKinObjPtr);
// serialHandler serialObj = *serialObjPtr;

ps2Handler *ps2ObjPtr = new ps2Handler(*allInterrupts::leftMotorPtr, *allInterrupts::rightMotorPtr, *robotKinObjPtr);
// ps2Handler ps2Obj = *ps2ObjPtr;

/// @brief Left motor job loop
void leftMotorJob()
{
  leftMotor.pidObj->setCurrentPositionCount(leftMotor.current_position_motor);
  leftMotor.motorCalculateShaftSpeed();
  if (!robotKinObjPtr->getIsRobotStop())
  {
    leftMotor.motorCalculatePid();
  }
}

/// @brief Right motor job loop
void rightMotorJob()
{
  rightMotor.pidObj->setCurrentPositionCount(rightMotor.current_position_motor);
  rightMotor.motorCalculateShaftSpeed();
  if (!robotKinObjPtr->getIsRobotStop())
  {
    rightMotor.motorCalculatePid();
  }
}

void serailJob()
{
  serialObjPtr->readSerialData();
  // serialObj.printEncoderCounts();
  serialObjPtr->printSpeeds(leftMotor.pidObj->getMappedControlVariable(), rightMotor.pidObj->getMappedControlVariable());
}

void ps2Job()
{
  ps2ObjPtr->readPs2Data();
  // robotKinObjPtr->printKinematicsData();
  //  serialObj.printEncoderCounts();
  serialObjPtr->printSpeeds(leftMotor.pidObj->getMappedControlVariable(), rightMotor.pidObj->getMappedControlVariable());
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial.println("Setup Start!");
  leftMotor.motorStop();
  rightMotor.motorStop();
  allInterrupts::attachInterruptFuncs();
  Serial.println("Setup Done!");
}

void loop()
{
  leftMotorJob();
  rightMotorJob();
  // serailJob();
  ps2Job();
}

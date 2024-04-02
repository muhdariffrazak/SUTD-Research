#include "DynamixelCleaningMotor.h"
#include <Arduino.h>
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

DynamixelCleaningMotor::DynamixelCleaningMotor(uint8_t MotorID, unsigned long MotorBaud, float Protocol, char side)
{
  //constructor
  this->MotorID = MotorID;
  this->MotorBaud = MotorBaud;
  this->Protocol = Protocol; 
  //numOfMotors++;
}

void DynamixelCleaningMotor::initCleaningMotor()
{
  
  Dynamixel2Arduino::begin(MotorBaud);
  Dynamixel2Arduino::setPortProtocolVersion(Protocol);
  Dynamixel2Arduino::ping(MotorID);


  DynamixelCleaningMotor::torqueOff(MotorID);
  DynamixelCleaningMotor::setOperatingMode(MotorID, OP_POSITION);
  DynamixelCleaningMotor::torqueOn(MotorID);
}

void DynamixelCleaningMotor::cleanMode()
{

  // Set Goal Position in DEGREE value
  DynamixelCleaningMotor::setGoalPosition(MotorID, 150.0, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");//shift to main code
  //DEBUG_SERIAL.println(DynamixelCleaningMotor::getPresentPosition(MotorID, UNIT_DEGREE));//shift to main code
  delay(1000);
}

void DynamixelCleaningMotor::climbMode()
{

  // Set Goal Position in DEGREE value
  DynamixelCleaningMotor::setGoalPosition(MotorID, 240, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");
  //DEBUG_SERIAL.println(DynamixelCleaningMotor::getPresentPosition(MotorID, UNIT_DEGREE));
  delay(1000);
}

void DynamixelCleaningMotor::SafeStartMotor()
{
  if (DynamixelCleaningMotor::getPresentPosition(MotorID, UNIT_DEGREE) <= 240.0)
  {
    //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");
    //DEBUG_SERIAL.println(DynamixelCleaningMotor::getPresentPosition(MotorID, UNIT_DEGREE));
    delay(1000);
  }
  else
  {
    DynamixelCleaningMotor::climbMode();
  }
}

void DynamixelCleaningMotor::moveMotor(int position)
{
  DynamixelCleaningMotor::setGoalPosition(MotorID, position);
  //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");
  //DEBUG_SERIAL.println(DynamixelCleaningMotor::getPresentPosition(MotorID, UNIT_DEGREE));
  delay(1000);
}

/*static void ShowNumOfMotors()
{
  return DynamixelCleaningMotor::numOfMotors;
}*/
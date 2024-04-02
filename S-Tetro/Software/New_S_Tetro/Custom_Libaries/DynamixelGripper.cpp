#include "DynamixelGripper.h"
#include <Arduino.h>
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>


//include below in main prog



DynamixelGripper::DynamixelGripper( uint8_t GripperID, unsigned long GripperBaud, float Protocol)
{
  //constructor
  this->GripperID = GripperID;
  this->GripperBaud = GripperBaud;
  this->Protocol = Protocol; 
  //numOfGrippers++;
}

void DynamixelGripper::initGripper()
{
  
  Dynamixel2Arduino::begin(GripperBaud);
  Dynamixel2Arduino::setPortProtocolVersion(Protocol);
  Dynamixel2Arduino::ping(GripperID);


  DynamixelGripper::torqueOff(GripperID);
  DynamixelGripper::setOperatingMode(GripperID, OP_POSITION);
  DynamixelGripper::torqueOn(GripperID);
}

void DynamixelGripper::openGripper()
{

  // Set Goal Position in DEGREE value
  DynamixelGripper::setGoalPosition(GripperID, 290.0, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");//shift to main code
  //DEBUG_SERIAL.println(DynamixelGripper::getPresentPosition(GripperID, UNIT_DEGREE));//shift to main code
  delay(1000);
}

void DynamixelGripper::closeGripper()
{

  // Set Goal Position in DEGREE value
  DynamixelGripper::setGoalPosition(GripperID, 270.0, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");
  //DEBUG_SERIAL.println(DynamixelGripper::getPresentPosition(GripperID, UNIT_DEGREE));
  delay(1000);
}

void DynamixelGripper::SafeStartGripper()
{
  if (DynamixelGripper::getPresentPosition(GripperID, UNIT_DEGREE) <= 290.0)
  {
    //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");
    //DEBUG_SERIAL.println(DynamixelGripper::getPresentPosition(GripperID, UNIT_DEGREE));
    delay(1000);
  }
  else
  {
    DynamixelGripper::openGripper();
  }
}

void DynamixelGripper::moveGripper(int position)
{
  DynamixelGripper::setGoalPosition(GripperID, position);
  //DEBUG_SERIAL.print("Present Position(degree) for Gripper : ");
  //DEBUG_SERIAL.println(DynamixelGripper::getPresentPosition(GripperID, UNIT_DEGREE));
  delay(1000);
}

static int ShowNumOfGrippers(){
  return DynamixelGripper::numOfGrippers;
}
#ifndef DynamixelGripper_h
#define DynamixelGripper_h

#include <Arduino.h>
#include <DynamixelShield.h>

//Type in class definition here
class DynamixelGripper: public DynamixelShield 
{
public:
//constructor
DynamixelGripper(uint8_t GripperID, unsigned long GripperBaud, float Protocol);

//static member
static int numOfGrippers;
//class functions
void initGripper();
void openGripper();
void closeGripper();
void SafeStartGripper();
void moveGripper(int angle);
static int ShowNumOfGrippers();

private:
uint8_t GripperID;
unsigned long GripperBaud;
float Protocol;
float angle;


};




//End of class definition



#endif
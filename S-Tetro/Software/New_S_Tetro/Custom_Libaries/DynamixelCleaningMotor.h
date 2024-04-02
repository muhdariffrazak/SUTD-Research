#ifndef DynamixelCleaningMotor_h
#define DynamixelCleaningMotor_h
#include <Arduino.h>
#include <DynamixelShield.h>
#include <SoftwareSerial.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

//Type in class definition here
class DynamixelCleaningMotor: public DynamixelShield 
{
public:
//constructor
DynamixelCleaningMotor(uint8_t MotorID, unsigned long MotorBaud, float Protocol, char side);

//static member
static int numOfMotors;
//class functions
void initCleaningMotor();
void cleanMode();
void climbMode();
void SafeStartMotor();
void moveMotor(int angle);
static int ShowNumOfMotors();

private:
uint8_t MotorID;
unsigned long MotorBaud;
float Protocol;
float angle;
char side;

};




//End of class definition



#endif
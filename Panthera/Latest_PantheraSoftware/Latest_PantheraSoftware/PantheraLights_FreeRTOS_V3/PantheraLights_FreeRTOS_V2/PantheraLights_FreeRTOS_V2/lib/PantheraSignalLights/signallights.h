#ifndef signallights_h
#define signallights_h

#include <Arduino.h>
#include <FastLED.h>
#include <Arduino_FreeRTOS.h>


//Type in class definition here

class signallights
{
public:
//constructor
signallights(unsigned long interval);
//class functions

void init_signallights();
void LeftHardSignalOn();
void signalOff();
void Autonomous();
void AutonomousTurnRight();
void AutonomousTurnLeft();
void RightHardSignalOn();
void TeleOp();
void TeleOpTurnLeft();
void TeleOpTurnRight();
void HazardLightOn();



//Encapsulating motor data in private
private:

static const unsigned int LED_PIN;
unsigned long interval;

};

#endif
#ifndef signallight_h
#define signallight_h

#include <Arduino.h>
#include <FastLED.h>
#include <Arduino_FreeRTOS.h>





//Type in class definition here

class signallights
{
public:
//constructor
signallight(uint8_t LED_PIN, unsigned long interval);
//class functions

void init_signallight();
void HardSignalOn();
void signalOff();
void SoftSignalOn();


//Encapsulating motor data in private
private:

static const unsigned int LED_PIN;
unsigned long interval;

};

#endif
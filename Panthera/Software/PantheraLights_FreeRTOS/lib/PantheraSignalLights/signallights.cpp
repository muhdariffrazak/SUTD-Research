#include "signallights.h"
#include <Arduino.h>
#include <FastLED.h>
#include <Arduino_FreeRTOS.h>

#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 10
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

//Constructor
signallights::signallights(unsigned long interval){
  this->interval = interval;
}

// Initialize the signallights
void::signallights::init_signallights(){
  // tell FastLED there's 60 NEOPIXEL leds on pin 2
  FastLED.addLeds<NEOPIXEL, 8>(leds[0], NUM_LEDS_PER_STRIP);
  // tell FastLED there's 60 NEOPIXEL leds on pin 3
  FastLED.addLeds<NEOPIXEL, 2>(leds[1], NUM_LEDS_PER_STRIP);
  signallights::signalOff();
}





//-------------------- Left Signal Lights--------------------

void signallights::LeftHardSignalOn(){
for (int i = 0; i <= 9; i++) {
  leds[0][i] = CRGB::OrangeRed;
  FastLED.show();
  vTaskDelay( interval / portTICK_PERIOD_MS );
}
signalOff();
}

void signallights::LeftSoftSignalOn(){
  for (int j =0; j<=2 ;j++){
    for (int i = 0; i <= 9; i++) {
      leds[0][i] = CRGB::OrangeRed;
      FastLED.show();
      vTaskDelay( interval / portTICK_PERIOD_MS );
    }
    signalOff();
  }
}
//-------------------- End of Left Signal Lights--------------------


//-------------------- Right Signal Lights--------------------
void signallights::RightHardSignalOn(){
  for (int i = 0; i <= 9; i++) {
    leds[1][i] = CRGB::OrangeRed;
    FastLED.show();
    vTaskDelay( interval / portTICK_PERIOD_MS );
  }
  signalOff();
}

void signallights::RightSoftSignalOn(){
  for (int j =0; j<=2 ;j++){
    for (int i = 0; i <= 9; i++) {
      leds[1][i] = CRGB::OrangeRed;
      FastLED.show();
      vTaskDelay( interval / portTICK_PERIOD_MS );
    }
    signalOff();
  }
}
//-------------------- End of Right Signal Lights--------------------


//-------------------- Hazard Signal Lights--------------------
void signallights::HazardLightOn(){
for (int i = 0; i <= 9; i++) {
  leds[0][i] = CRGB::OrangeRed;
  leds[1][i] = CRGB::OrangeRed;
  FastLED.show();
  vTaskDelay( interval / portTICK_PERIOD_MS );
}
signalOff();
}
//-------------------- End of Hazard Signal Lights--------------------


//-------------------- Switch Off Signal Lights--------------------
void signallights::signalOff(){
  FastLED.clear();  // clear all pixel data
  FastLED.show();
  vTaskDelay(interval / portTICK_PERIOD_MS);
}
// -------------------- Switch Off Signal Lights--------------------



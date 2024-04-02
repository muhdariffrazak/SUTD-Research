#include "signallights.h"
#include <Arduino.h>
#include <FastLED.h>
#include <Arduino_FreeRTOS.h>

#define NUM_LEDS    10
CRGB leds[NUM_LEDS];

//Constructor

signallight::signallight(uint8_t LED_PIN , unsigned long interval)
{
  this->interval = interval;
}

void::signallight::init_signallight(){
  FastLED.addLeds<WS2812B, LED_PIN , GRB>(leds, NUM_LEDS);
  signallight::signalOff();
}

void signallight::HardSignalOn(){
for (int i = 0; i <= 9; i++) {
  leds[i] = CRGB::OrangeRed;
  FastLED.show();
  vTaskDelay( 100 / portTICK_PERIOD_MS );
}
  FastLED.clear();  // clear all pixel data
  FastLED.show();
  vTaskDelay( 100 / portTICK_PERIOD_MS );
}

void signallight::signalOff(){
  FastLED.clear();  // clear all pixel data
  FastLED.show();
  vTaskDelay(interval / portTICK_PERIOD_MS);
}

void signallight::SoftSignalOn(){
  for (int j =0; j<=2 ;j++){
    for (int i = 0; i <= 9; i++) {
      leds[i] = CRGB::OrangeRed;
      FastLED.show();
      vTaskDelay( 100 / portTICK_PERIOD_MS );
    }
    signalOff();
  }
}
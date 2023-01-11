// Test motor joint mode

#include "DynamixelMotor.h"

// id of the motor
const uint8_t id2=2,id3=3;
// speed, between 0 and 1023
int16_t speed=512;
// communication baudrate
const long unsigned int baudrate = 1000000;

// hardware serial without tristate buffer
// see blink_led example, and adapt to your configuration
HardwareDynamixelInterface interface2(Serial);
HardwareDynamixelInterface interface3(Serial);


DynamixelMotor motor2(interface2, id2);
DynamixelMotor motor3(interface2, id3);

void setup()
{ 
  interface2.begin(baudrate);
  delay(100);
  interface3.begin(baudrate);
  delay(100);
  
  // check if we can communicate with the motor
  // if not, we turn the led on and stop here
  uint8_t status2=motor2.init();
  if(status2!=DYN_STATUS_OK)
  {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    while(1);
  }

  uint8_t status3=motor3.init();
  if(status3!=DYN_STATUS_OK)
  {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    while(1);
  }

  motor2.enableTorque();  

  motor3.enableTorque();  

  // set to joint mode, with a 180° angle range
  // see robotis doc to compute angle values
  motor2.jointMode(204, 820);
  motor2.speed(speed);

  motor3.jointMode(204, 820);
  motor3.speed(speed);
}

void loop() 
{
  // go to middle position
  motor2.goalPosition(512);
  delay(500);

  motor3.goalPosition(512);
  delay(500);



  // move 45° CCW
  motor2.goalPosition(666);
  delay(500);

  motor3.goalPosition(666);
  delay(500);



  // go to middle position
  motor2.goalPosition(512);
  delay(500);

  motor3.goalPosition(512);
  delay(500);


  // move 45° CW
  motor2.goalPosition(358);
  delay(500);

  motor3.goalPosition(358);
  delay(500);
}



#include <DynamixelCleaningMotor.h>
#include <DynamixelGripper.h>
#include <LinearActuator.h>


#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>


#include <SparkFun_BNO080_Arduino_Library.h>


/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the parts of the calibrated gyro.

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
DynamixelGripper A(1, 57600, 2.0);
byte GripperClose=0;

void setup()
{
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("BNO080 Read Example");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();

  myIMU.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  
  myIMU.enableRotationVector(50); //Send data update every 50ms


  DEBUG_SERIAL.println(F("Rotation vector enabled"));
  DEBUG_SERIAL.println(F("Output in form roll, pitch, yaw"));
  A.initGripper();
  A.SafeStartGripper();



}


void runCheck(){
  if (myIMU.dataAvailable() == true)
  {
    float roll = (myIMU.getRoll()) * 180.0 / PI;
    DEBUG_SERIAL.print(roll, 1);
  
    DEBUG_SERIAL.println();
  }
}

void CheckIfFall(){
  if (abs((myIMU.getRoll()) * 180.0 / PI)> 5.5){
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println("FALLLLL");
    if (GripperClose==0){
    A.closeGripper();
    }
    DEBUG_SERIAL.println((A.getPos()));
  
    }
}


void CheckIfStable(){
  if (abs((myIMU.getRoll()) * 180.0 / PI)< 5.5){
    digitalWrite(LED_BUILTIN, HIGH);
    A.openGripper();
  
    }
}

void loop()
{
  //Look for reports from the IMU
  runCheck();
  CheckIfFall();
  //CheckIfStable();
}


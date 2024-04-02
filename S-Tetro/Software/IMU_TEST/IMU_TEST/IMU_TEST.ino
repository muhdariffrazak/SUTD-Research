#include <Wire.h>
#include <HardwareSerial.h>

#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 IMU DATA");

  Wire.setClock(400000);  //Increase I2C data rate to 400kHz

  myIMU.begin();

  Wire.setClock(400000);  //Increase I2C data rate to 400kHz

  myIMU.enableAccelerometer(50);  //Send data update every 50ms
  myIMU.enableGyro(50);           //Send data update every 50ms

  Serial.println(F("Accelerometer enabled"));
  Serial.println(F("Output in form x, y, z, in m/s^2"));
}


void loop() {
  // put your main code here, to run repeatedly:
  if (myIMU.dataAvailable() == true) {
    Accelerometer();
    Gyroscope();
  }
}

void Accelerometer() {
  float x = myIMU.getAccelX();
  float y = myIMU.getAccelY();
  float z = myIMU.getAccelZ();

  Serial.println(F("Accelerometer in m/s^2"));
  Serial.print(x, 2);
  Serial.print(F(","));
  Serial.print(y, 2);
  Serial.print(F(","));
  Serial.print(z, 2);
  Serial.print(F(",\n"));
  Serial.println(F("----------------------------"));

  delay(500);
}

void Gyroscope() {

  float x = myIMU.getGyroX();
  float y = myIMU.getGyroY();
  float z = myIMU.getGyroZ();

  Serial.println(F("Gyroscope is in radians per second"));
  Serial.print(x, 2);
  Serial.print(F(","));
  Serial.print(y, 2);
  Serial.print(F(","));
  Serial.print(z, 2);
  Serial.print(F(",\n"));
  Serial.println(F("----------------------------"));

  delay(500);
}
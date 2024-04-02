#include <Wire.h>
#include <HardwareSerial.h>

bool cal 

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 IMU DATA");

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.begin();

  myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag

  myIMU.enableAccelerometer(50); //Send data update every 50ms
  myIMU.enableGyro(50); //Send data update every 50ms

  Serial.println(F("IMU_DATA enabled. Press 'c' to save to recalibrate"));
  Serial.println(F("Output in form x, y, z, in m/s^2"));
}


void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    byte incoming = Serial.read();

    if(incoming == 'c')
    {
      myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag
      cal=True;
      myIMU.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
      myIMU.requestCalibrationStatus(); //Sends command to get the latest calibration status

      //Wait for calibration response, timeout if no response
      int counter = 100;
      while(1)
      {
        if(--counter == 0) break;
        if(myIMU.dataAvailable() == true)
        {
          //The IMU can report many different things. We must wait
          //for the ME Calibration Response Status byte to go to zero
          if(myIMU.calibrationComplete() == true)
          {
            Serial.println("Calibration data successfully stored");
            delay(1000);
            break;
          }
        }

        delay(1);
      }
      if(counter == 0)
      {
        Serial.println("Calibration data failed to store. Please try again.");
      }

      //myIMU.endCalibration(); //Turns off all calibration
      //In general, calibration should be left on at all times. The BNO080
      //auto-calibrates and auto-records cal data roughly every 5 minutes
    }
  }
   if (myIMU.dataAvailable() == true){
    Accelerometer();
    Gyroscope();}

}

void Accelerometer(){
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

void Gyroscope(){
  
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
#include <Arduino.h>
#include <Wire.h>

bool cal = false; // Calibration flag. Set to true when 'c' is pressed
unsigned long start_time;
unsigned long cal_time = (3000);


#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

// Function prototypes
void Accelerometer();
void Gyroscope();
void printAccuracyLevel(byte accuracyNumber);
void getCalibrationdata();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 IMU DATA");

  Wire.setClock(400000); // Increase I2C data rate to 400kHz

  myIMU.begin();

  myIMU.enableAccelerometer(50); // Send data update every 50ms
  myIMU.enableGyro(50);          // Send data update every 50ms

  Serial.println(F("IMU_DATA enabled. Press 'c' to save to recalibrate"));
  Serial.println(F("Output in form x, y, z, in m/s^2"));
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'c')
    {
      myIMU.calibrateAll(); // Turn on cal for Accel, Gyro, and Mag
      myIMU.enableGameRotationVector(100); //Send data update every 100ms
      myIMU.enableMagnetometer(100); //Send data update every 100ms

      cal = true;
      Serial.println("Calibrating");
      start_time = millis();
    }
  }
  if (myIMU.dataAvailable() && cal)
  {
    while (millis() - start_time < cal_time)
    {
      getCalibrationdata();
    }
    myIMU.saveCalibration();     // Saves the current dynamic calibration data (DCD) to memory
    myIMU.requestCalibrationStatus(); // Sends command to get the latest calibration status
    int counter = 100;
    while (1)
    {
      if (--counter == 0)
        break;
      if (myIMU.dataAvailable())
      {
        // The IMU can report many different things. We must wait
        // for the ME Calibration Response Status byte to go to zero
        if (myIMU.calibrationComplete())
        {
          Serial.println("Calibration data successfully stored");
          cal = false;
          setup(); // Re-initialize the IMU
          delay(1000);
          break;
        }
      }

      delay(1);
    }
    if (counter == 0)
    {
      Serial.println("Calibration data failed to store. Please try again.");
      cal = false;
      setup(); // Re-initialize the IMU
    }
  }

  if (myIMU.dataAvailable() && !cal)
  {
    Accelerometer();
    Gyroscope();
  }
}

void Accelerometer()
{
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

void Gyroscope()
{
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

void printAccuracyLevel(byte accuracyNumber)
{
  if (accuracyNumber == 0)
    Serial.print(F("Unreliable"));
  else if (accuracyNumber == 1)
    Serial.print(F("Low"));
  else if (accuracyNumber == 2)
    Serial.print(F("Medium"));
  else if (accuracyNumber == 3)
    Serial.print(F("High"));
}

void getCalibrationdata()
{
  float x = myIMU.getMagX();
  float y = myIMU.getMagY();
  float z = myIMU.getMagZ();
  byte accuracy = myIMU.getMagAccuracy();

  float quatI = myIMU.getQuatI();
  float quatJ = myIMU.getQuatJ();
  float quatK = myIMU.getQuatK();
  float quatReal = myIMU.getQuatReal();
  byte sensorAccuracy = myIMU.getQuatAccuracy();

  Serial.print(x, 2);
  Serial.print(F(","));
  Serial.print(y, 2);
  Serial.print(F(","));
  Serial.print(z, 2);
  Serial.print(F(","));
  printAccuracyLevel(accuracy);
  Serial.print(F(","));

  Serial.print("\t");

  Serial.print(quatI, 2);
  Serial.print(F(","));
  Serial.print(quatJ, 2);
  Serial.print(F(","));
  Serial.print(quatK, 2);
  Serial.print(F(","));
  Serial.print(quatReal, 2);
  Serial.print(F(","));
  printAccuracyLevel(sensorAccuracy);
  Serial.print(F(","));

  Serial.println();
}

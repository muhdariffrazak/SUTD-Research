#include <Arduino.h>
#include <Wire.h>

bool cal = false; // Calibration flag. Set to true when 'c' is pressed
unsigned long start_time;
unsigned long curr_time;


#include <SoftwareSerial.h>

SoftwareSerial hc06(2,3); // RX, TX

String cmd="";

String data="";
String comma = ",";

bool start = false;


#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

// Function prototypes
void get_time();
void Accelerometer();
void Gyroscope();
void printAccuracyLevel(byte accuracyNumber);
void getCalibrationdata();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  hc06.begin(115200);

  Serial.println("Starting IMU");
  
  //hc06.println();
  //hc06.println("BNO080 IMU DATA");

  Wire.setClock(400000); // Increase I2C data rate to 400kHz

  myIMU.begin();

  myIMU.enableAccelerometer(25); // Send data update every 50ms
  myIMU.enableGyro(25);          // Send data update every 50ms

  //hc06.println(F("IMU_DATA enabled. Press 'c' to save to recalibrate"));
  //hc06.println(F("Output in form x, y, z, in m/s^2"));
}
//
void loop()
{
  while(hc06.available()>0){
    if (start==false){
      start_time=millis();
      start=true;
    }
  }

  if (myIMU.dataAvailable() && !cal)
  {
    get_time();
    Accelerometer();
    Gyroscope();
    Serial.print(data);
    hc06.println(data);
    data="";
  }
}

void get_time(){
  curr_time=millis()-start_time;
  unsigned long seconds = curr_time  / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  curr_time %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;                                        //Remove the number of hours and minutes, leaving only seconds.
  String hrMinSec = (String(minutes) + ":" + String(seconds) + ":" + String(curr_time));  //Converts to HH:MM:SS string. This can be returned to the calling function.
  data = String(hrMinSec) + comma;
}

void Accelerometer()
{
  float x = myIMU.getAccelX();
  float y = myIMU.getAccelY();
  float z = myIMU.getAccelZ();
  data += String(x,2) + comma + String(y,2) + comma + String(z,2) + comma;
  delay(1);
}
void Gyroscope()
{
  float x = myIMU.getGyroX();
  float y = myIMU.getGyroY();
  float z = myIMU.getGyroZ();

  data += String(x,2) + comma + String(y,2) + comma + String(z,2) + comma;
  delay(1);
}


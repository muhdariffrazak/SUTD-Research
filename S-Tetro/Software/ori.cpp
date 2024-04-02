#include <Arduino.h>

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"
#include "RoboClaw.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

RoboClaw roboclaw(&Serial2, 10000);
//RoboClaw roboclaw2(&Serial3, 10000);
#define address1 0x80 //128
//define address2 0x81 //129
//#define address3 0x82 //130*

#define pi 3.14
#define gear_ratio 150 //Gear ratio of the motor
#define Radius 0.035 //Radius of the wheel
#define L 0.225 //Distance between the wheels in m
#define B 0.3 //Distance between the wheels in m
#define omega 0.0 //Angular velocity of the robot around z axis in rad/s

#define Kp_fL 14725.25293
#define Ki_1R 1087.66882
#define Kd_1R 0.0
#define qpps_1R 3300
#define Kp_1L 16046.74902
#define Ki_1L 1196.89490
#define Kd_1L 0.0
#define qpps_1L 3300

#define Kp_2R 14725.25293
#define Ki_2R 1087.66882
#define Kd_2R 0.0
#define qpps_2R 3300
#define Kp_2L 16046.74902
#define Ki_2L 1196.89490
#define Kd_2L 0.0
#define qpps_2L 3300

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  //Serial2.begin(115200);
  roboclaw.begin(38400);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    //Serial.write(SerialBT.read());

    char i = Serial.read();

      if (i == 'F')
      {
        roboclaw.SpeedM1(address1, 4480);
        roboclaw.SpeedM2(address1, 4480);
        //roboclaw.SpeedM1(address2, 4480);
        //roboclaw.SpeedM2(address2, 4480);
        Serial.println("Forward");
        //roboclaw2.SpeedM1(address3,2000);
        //roboclaw2.SpeedM2(address3,2000);
        //Serial2.println("Echo");
      }

      



      /*else if (i == 'B')
      {

        
        roboclaw.SpeedM1(address1, 4480);
        roboclaw.SpeedM2(address1, int(vel_fl*4480));
        roboclaw.SpeedM1(address2, int(vel_br*4480));
        roboclaw.SpeedM2(address2, int(vel_bl*4480));
        roboclaw2.SpeedM1(address3, -2000);
        roboclaw2.SpeedM2(address3, -2000);
        

      }

      else if (i == 'R')
      {


        roboclaw.SpeedM1(address1, int(vel_fr*4480));
        roboclaw.SpeedM2(address1, int(vel_fl*4480));
        roboclaw.SpeedM1(address2, int(vel_br*4480));
        roboclaw.SpeedM2(address2, int(vel_bl*4480));
        roboclaw2.SpeedM1(address3, 0);
        roboclaw2.SpeedM2(address3, 0);

      }

      else if (i == 'L')
      {


          roboclaw.SpeedM1(address1, int(vel_fr*4480));
          roboclaw.SpeedM2(address1, int(vel_fl*4480));
          roboclaw.SpeedM1(address2, int(vel_br*4480));
          roboclaw.SpeedM2(address2, int(vel_bl*4480));
          roboclaw2.SpeedM1(address3, 0);
          roboclaw2.SpeedM2(address3, 0);
        
        



        
      }*/

  }
  delay(20);
}


void move(float v_x, float v_y)//v_x and v_y are the velocities in x and y direction respectively
{    
    float v_lx= v_x;
    float v_ly= v_y;
    float v_w= omega*0.020;

    float vel_fl =((1/Radius)*(v_lx-v_ly-(L+B)*v_w))*(60/(pi*gear_ratio));
    float vel_fr =((1/Radius)*(v_lx+v_ly+(L+B)*v_w))*(60/(pi*gear_ratio));
    float vel_bl =((1/Radius)*(v_lx+v_ly-(L+B)*v_w))*(60/(pi*gear_ratio));
    float vel_br =((1/Radius)*(v_lx-v_ly+(L+B)*v_w))*(60/(pi*gear_ratio));

    //roboclaw.SpeedM1(address1, int(vel_fr*4480));
    //roboclaw.SpeedM2(address1, int(vel_fl*4480));
    //roboclaw.SpeedM1(address2, int(vel_br*4480));
    //roboclaw.SpeedM2(address2, int(vel_bl*4480));
    //roboclaw2.SpeedM1(address3, 0);
    //roboclaw2.SpeedM2(address3, 0);
}

#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "RoboClaw.h"
#include <AccelStepper.h>
#include <SparkFun_VL6180X.h>
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>



SFEVL53L1X distanceSensor;
#define VL6180X_ADDRESS 0x29
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address

VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

RoboClaw roboclaw(&Serial1, 10000);
RoboClaw roboclaw2(&Serial2, 10000);
#define address1 0x80 //128
#define address2 0x81 //129
#define address3 0x82 //130

#define NUMBER_OF_SENSORS 3

// Step 1 -  Instantiating an object library


double eTheta, deTheta, omega;
double eThetaP = 0;

double eDis, deDis, Vx;
double eDisP = 0;

int Brake_1 = 51;
int Brake_2 = 53;

int F_Down = 49;
int F_Up = 47;
int M_Down = 45;
int M_Up = 43;
int B_Down = 41;
int B_Up = 39; 

int Limit_1 = 35;
int Limit_2 = 37;

float R = 0.03;
float L = 0.22;
float B = 0.32;

int f_state=0;
int RL_flag=0;
int cmd_vel_flag = 0;
char i;
char p_i;
int p_i_2;
AccelStepper stepper;

#define Kp_1R 14725.25293
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

uint32_t prevMillis;
uint32_t prevMillis_2;
uint32_t PrevMillis_3;
uint32_t PrevMillis_4;
uint32_t PrevMillis_5;
uint32_t PrevOdomMillis;
int timer_flag = 0;
int timer_flag_inner = 0;
int timer_longstep = 0; 
int long_step_detect = 0;
bool led_state = false;
int inner_loop_timer = 0;

int floor_detect = 0;
int Fsv = 0;
int Msv = 0;
int Bsv = 0;
int Rsv = 0;
int Lsv = 0;

double x1 = 0;
double y1 = 0;
double x2 = 280;
double y2 = 0;
double angle =0;
double angle_2=0;

bool step_detection;

float step_x=0.0;
float step_y=0.0;
float step_theta=0.0;

float x_pos;
float y_pos;
float theta;

//Enables a specific port number



void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  delay(500);
}

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial3.print("\t");
  if (!system)
  {
    Serial3.print("! ");
  }



}



void setup() {
  // put your setup code here, to run once:
  //nh.initNode();//ROS
 // nh.subscribe(sub);//ROS
  //nh.advertise(stetro_status_msg);
  Serial.begin(57600);
  Serial3.begin(9600);

  //Wire.begin();

  delay(100);

 
  /* Initialise the sensor */
  //if (!bno.begin())
  //{
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial3.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  //}
  

  delay(100);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Optional: Display current status */
  //displaySensorStatus();
  
  //displayCalStatus();
  
  //bno.setExtCrystalUse(true);

  roboclaw.begin(115200);
  delay(100);
  roboclaw2.begin(115200);



  pinMode (Brake_1, OUTPUT);
  pinMode (Brake_2, OUTPUT);

  pinMode (13, OUTPUT);

  pinMode (F_Down, OUTPUT);
  pinMode (F_Up, OUTPUT);
  pinMode (M_Down, OUTPUT);

  pinMode (M_Up, OUTPUT);
  pinMode (B_Down, OUTPUT);
  pinMode (B_Up, OUTPUT);

  pinMode (Limit_1, OUTPUT);
  pinMode (Limit_2, OUTPUT);
  
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM1(address2, 0);
  roboclaw.SpeedM2(address2, 0);

  digitalWrite(Brake_1, HIGH);
  digitalWrite(Brake_2, HIGH);

  digitalWrite(Limit_1, LOW);
  digitalWrite(Limit_2, LOW);
  
  prevMillis = millis();
  prevMillis_2 = 0;
  timer_flag = 0;


 
 /////////////////////////////////////////////////////////
}

void loop() {

 // sensors_event_t event;
 // bno.getEvent(&event);


  
//  if(event.orientation.x>=180.0)
//  {
//  }
//  else
//  {

//  }
  



  
  if (Serial3.available() > 0)
    i = Serial3.read();
 
  if (i == 'F')
  {
 
    roboclaw.SpeedM1(address1, 4480);
    roboclaw.SpeedM2(address1, 4480);
    roboclaw.SpeedM1(address2, 4480);
    roboclaw.SpeedM2(address2, 4480);
    roboclaw2.SpeedM1(address3,2000);
    roboclaw2.SpeedM2(address3,2000);
    


      


  }

  else if (i == 'B')
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
    
    



    
  }
  

  else if (i == 'Z')
  {

    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, 2000);
    roboclaw2.SpeedM2(address3, -2000);
     

  }

  else if (i == 'Q')
  { 

    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, 2000);
    roboclaw2.SpeedM2(address3, -2000);
    
  }

  else if (i == 'E')
  {

    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, -2000);
    roboclaw2.SpeedM2(address3, 2000);
  }

  else if (i == 'D')
  {
    if (timer_flag == 0)
    {
      prevMillis_2 = millis();
      timer_flag = 1;
      digitalWrite(Brake_1, LOW);
    }
    
    if((millis()-prevMillis_2>100) and (floor_detect == 0))
    {
    digitalWrite(F_Down, HIGH);
    enableMuxPort(0);
    Fsv = sensor.getDistance();
    disableMuxPort(0);
    Serial3.print(Fsv);
    Serial3.println("");
    if(Fsv < 20 and floor_detect == 0)
    {
      floor_detect = 1;
      timer_flag =0;
      digitalWrite(F_Down, LOW);
    }
    }
    
    else if((millis()-prevMillis_2>100)and(floor_detect == 1))
    {
      digitalWrite(Brake_1, HIGH);
      if(millis()-prevMillis_2>200)
      {
      floor_detect = 0;
      i = 'F';
      timer_flag =0;
      }
    }

    p_i ='D';
  }

  else if (i == 'U')
  {
    if (timer_flag == 0)
    {
      prevMillis_2 = millis();
      timer_flag = 1;
      digitalWrite(Brake_1, LOW);
      delay(100);
    }
    digitalWrite(F_Up, HIGH);
    if(digitalRead(Limit_1) == HIGH)
    {
      timer_flag=0;
      i = 'S';
      digitalWrite(F_Up, LOW);
      delay(50);
      digitalWrite(Brake_1, HIGH);
    }
  }

  else if (i == 'M')
  {
    if (timer_flag == 0)
    {
      prevMillis_2 = millis();
      timer_flag = 1;
      digitalWrite(Brake_1, LOW);
      digitalWrite(Brake_2, LOW);
    }


    if((millis()-prevMillis_2>100) and (floor_detect == 0))
    {
    enableMuxPort(1);
    Msv = sensor.getDistance();
    disableMuxPort(1);
    
    if(digitalRead(Limit_1) == LOW)
    {
      digitalWrite(M_Down, HIGH); //digitalWrite(M_Down, HIGH);
    }
    
    else if(digitalRead(Limit_1) == HIGH)
    {
      digitalWrite(M_Down, LOW);
      if(timer_flag_inner ==0)
      {
        PrevMillis_5 = millis();
        timer_flag_inner = 1;
      }
      if((millis()-PrevMillis_5>300))
      {
        digitalWrite(B_Up, HIGH);
        digitalWrite(Brake_1, HIGH);
      }
      
      if((Msv<22) and (millis()-PrevMillis_5>400))
      {
      digitalWrite(Brake_1, HIGH);
      floor_detect = 1;
      timer_flag = 0;
      timer_flag_inner = 0;
     /// Serial3.println(Msv);
      digitalWrite(B_Up, LOW);
      }
    }
    }

    else if ((millis()-prevMillis_2>600) and (floor_detect == 1))
    {
      
      digitalWrite(Brake_2, HIGH);
      if(millis()-prevMillis_2>800)
      {
        i = 'F';
        floor_detect = 0;
        timer_flag = 0;
      }
      
    }
    p_i = 'M';
  }

  else if (i == 'N')
  {
    digitalWrite(Brake_1, LOW);
    digitalWrite(Brake_2, LOW);
    digitalWrite(M_Up, HIGH);
  }

  else if (i == 'T')
  {
    if (timer_flag == 0)
    {
      prevMillis_2 = millis();
      timer_flag = 1;
      digitalWrite(Brake_2, LOW);
    }

  if((millis()-prevMillis_2>100) and (floor_detect == 0))
  {
    enableMuxPort(6);
    Rsv = sensor.getDistance();
    disableMuxPort(6);
    delay(1);
    enableMuxPort(7);
    Lsv = sensor.getDistance();
    disableMuxPort(7);

    digitalWrite(B_Down, HIGH);
    
    if(digitalRead(Limit_2) == HIGH)
    {
      floor_detect = 1;
      timer_flag = 0;
      digitalWrite(B_Down, LOW);
    }
  }

  else if((millis()-prevMillis_2>500) and (floor_detect == 1))
  {
      digitalWrite(Brake_2, HIGH);
      
      if((millis()-prevMillis_2>700))
      {
      i = 'B';
      floor_detect = 0;
      timer_flag = 0;
    }
  }
    p_i = 'T';
  }
  
  else if (i == 'H')
  {
    if (timer_flag == 0)
    {
      prevMillis_2 = millis();
      timer_flag = 1;
      digitalWrite(Brake_2, LOW);
      delay(100);
    }
    digitalWrite(B_Up, HIGH);
  }
  else if (i == 'J')
  {
    digitalWrite(Brake_1, LOW);
    digitalWrite(Brake_2, LOW);
  }
  else
  {
    if(cmd_vel_flag ==0)
    {
    if(inner_loop_timer == 0){
      PrevMillis_4 = millis();
      inner_loop_timer = 1;
    }
    roboclaw.SpeedM1(address1, 0);
    roboclaw.SpeedM2(address1, 0);
    roboclaw.SpeedM1(address2, 0);
    roboclaw.SpeedM2(address2, 0);
    roboclaw2.SpeedM1(address3,0);
    roboclaw2.SpeedM2(address3,0);
    digitalWrite(F_Up, LOW);
    digitalWrite(F_Down, LOW);
    digitalWrite(M_Up, LOW);
    digitalWrite(M_Down, LOW);  
    digitalWrite(B_Up, LOW);
    digitalWrite(B_Down, LOW);

    if ((millis() - PrevMillis_4) > 400)
    {
    digitalWrite(Brake_1, HIGH);
    digitalWrite(Brake_2, HIGH);
    inner_loop_timer = 0;
    }
    timer_flag = 0;
    timer_longstep = 0;
    }
  }
  //delay(BNO055_SAMPLERATE_DELAY_MS);

  if ((millis() - prevMillis) > 500) {

    prevMillis = millis();

    if (led_state == false) {
      digitalWrite(13, HIGH);
      led_state = true;
    }

    else {
      digitalWrite(13, LOW);
      led_state = false;
    }
  }


}




#include <DynamixelCleaningMotor.h>
#include <DynamixelGripper.h>
#include <LinearActuator.h>


#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>


#include <SparkFun_BNO080_Arduino_Library.h>

//This few lines defines serial 1 to be used to output into a bluetooth module for feedback and control
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  #define DEBUG_SERIAL Serial1
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
#include <Wire.h>

BNO080 myIMU;
int DynamixelGripper::numOfGrippers=0;//Initialise num of grippers and linear actuators
int LinearActuator::numOfActuators=0;
DynamixelGripper A(1, 57600, 2.0);//Declare obj Gripper
LinearActuator Right(2,3,14,"R");//Declare obj linear act
LinearActuator Left(4,5,15,"L");//Declare obj linear act
byte GripperClose=0;//Flag to set Gripper as close
char command;

void setup()
{
  DEBUG_SERIAL.begin(9600);
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

//Initialise and safestart actuators
  A.initGripper();
  Right.initLinearActuator();
  Left.initLinearActuator();
  A.SafeStartGripper();
  Right.SafeStartActuator();
  Left.SafeStartActuator();
}

void loop()
{
  runCheck();
  CheckIfFall();

  if(DEBUG_SERIAL.available()>0){
    command=DEBUG_SERIAL.read();
  }
  if (command=='g'){
    DEBUG_SERIAL.println(Right.getPosition());
    DEBUG_SERIAL.println(Left.getPosition());
  }
  if (command=='r'){
    Right.Extend();
    DEBUG_SERIAL.println(Right.getPosition());
    Right.Contract();
    DEBUG_SERIAL.println(Right.getPosition());
  }
  if (command=='l'){
    Left.Extend();
    DEBUG_SERIAL.println(Left.getPosition());
    Left.Contract();
    DEBUG_SERIAL.println(Left.getPosition());
  }
  if (command == 'f'){
    Left.Contract();
    DEBUG_SERIAL.println(Left.getPosition());
    Right.Contract();
    DEBUG_SERIAL.println(Right.getPosition());
  }
  if (command == 'c'){
    ActuateTog();
  }
  if (command == 'o'){
     A.openGripper();
    delay(3000);
    GripperClose=0;
  }
  if  (command == 't'){
    FoldTog();
  }

  command = 0;


}

/*----------------------------------------------------
Gripper and IMU Functions*/
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
    Right.switchOff();
    Left.switchOff();
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println("FALLLLL");
    if (GripperClose==0){
    A.closeGripper();
    GripperClose=1;
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


/*----------------------------------------------------
Linear Actuator Functions*/
void ActuateTog(){
  int sensorValue = analogRead(A14);
  int sensorValue2 = analogRead(A13);
  while ((sensorValue < 800)&&(sensorValue2 < 800)){
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);

    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    
    DEBUG_SERIAL.println(Right.getPosition());
    DEBUG_SERIAL.println(Left.getPosition());
    sensorValue = analogRead(A14);
    sensorValue2 = analogRead(A13);
  }
  Right.switchOff();
  Left.switchOff();
}


void FoldTog(){
  int sensorValue = analogRead(A14);
  int sensorValue2 = analogRead(A13);
  while ((sensorValue > 125)&&(sensorValue2 > 125)){
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);

    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    
    DEBUG_SERIAL.println(Right.getPosition());
    DEBUG_SERIAL.println(Left.getPosition());
    sensorValue = analogRead(A14);
    sensorValue2 = analogRead(A13);
  }
  Right.switchOff();
  Left.switchOff();
}





















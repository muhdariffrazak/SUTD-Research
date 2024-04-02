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
#include "SparkFun_VL53L1X.h"
//#include <ComponentObject.h>
//#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
//#include <vl53l1x_class.h>
//#include <vl53l1_error_codes.h>
//#include <ros.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Point.h>
//#include <std_msgs/Char.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/Int32.h>
//#include <stetro_msgs/FirstStepPose.h>
//ros::NodeHandle  nh;
//#include <std_msgs/String.h>

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
Fuzzy* fuzzy = new Fuzzy();

Fuzzy* fuzzyDis = new Fuzzy();

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
boolean enableMuxPort(byte portNumber)
{
  if (portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if (!Wire.available()) return (false); //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return (true);
}

//Disables a specific port number
boolean disableMuxPort(byte portNumber)
{
  if (portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if (!Wire.available()) return (false); //Error
  byte settings = Wire.read();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return (true);
}


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

//void callback (const geometry_msgs::Twist& step_pose_)
/*{
  //step_detection = bool(step_pose_.is_detected);
  step_x = float(step_pose_.linear.x);
  step_y = float(step_pose_.linear.y);
  step_theta = -(step_pose_.linear.z); //(step_pose_.angular.z); //
}*/

//geometry_msgs::Twist stetro_msg ;
//ros::Publisher stetro_status_msg("stetro_msg", &stetro_msg);
//ros::Subscriber<geometry_msgs::Twist> sub("/first_step_pose", callback);

void setup() {
  // put your setup code here, to run once:
  //nh.initNode();//ROS
 // nh.subscribe(sub);//ROS
  //nh.advertise(stetro_status_msg);
  Serial.begin(57600);
  Serial3.begin(9600);

  Wire.begin();

  delay(100);

  for (byte x = 0 ; x < NUMBER_OF_SENSORS ; x++)
  {
    enableMuxPort(x); //Tell mux to connect to port X
    if (sensor.VL6180xInit() != 0) {
      Serial3.print(x);
      Serial3.println("FAILED TO INITALIZE"); //Initialize device and check for errors
    } //Init the sensor connected to this port
    sensor.VL6180xDefautSettings();
    disableMuxPort(x);
  }

  Serial3.println("Mux Shield online");
  //Serial.println("Orientation Sensor Test"); Serial.println("");
  enableMuxPort(3);
  sensor.VL6180xInit();
  sensor.VL6180xDefautSettings();
  disableMuxPort(3);
  enableMuxPort(5);
  sensor.VL6180xInit();
  sensor.VL6180xDefautSettings();
  disableMuxPort(5);
  enableMuxPort(6);
  sensor.VL6180xInit();
  sensor.VL6180xDefautSettings();
  disableMuxPort(6);
  enableMuxPort(7);
  sensor.VL6180xInit();
  sensor.VL6180xDefautSettings();
  disableMuxPort(7);
  enableMuxPort(4);
  sensor.VL6180xInit();
  sensor.VL6180xDefautSettings();
  disableMuxPort(4);
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial3.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(100);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();
  
  displayCalStatus();
  
  bno.setExtCrystalUse(true);

  roboclaw.begin(115200);
  delay(100);
  roboclaw2.begin(115200);
  stepper.setMaxSpeed(10000);
  stepper.setSpeed(10000);

  //  roboclaw.SetM1VelocityPID(address1,Kd,Kp,Ki,qpps);
  //  roboclaw.SetM2VelocityPID(address1,Kd2,Kp2,Ki2,qpps2);
  //  roboclaw.SetM1VelocityPID(address2,Kd,Kp,Ki,qpps);
  //  roboclaw.SetM2VelocityPID(address2,Kd2,Kp2,Ki2,qpps2);
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


   ////////////////////////////// Fuzzy Heading //////////////////////////////////
 // Step 2 - Creating a FuzzyInput E
 FuzzyInput* eThetaF = new FuzzyInput(1);// With its ID in param
 // Creating the FuzzySet to compond FuzzyInput E
 FuzzySet* NM_eThetaF = new FuzzySet(-1000, -1000, -1.0, -0.5); // 
 eThetaF->addFuzzySet(NM_eThetaF); // Add FuzzySet 
 FuzzySet* NS_eThetaF = new FuzzySet(-1.0, -0.5, -0.5, 0.0); // 
 eThetaF->addFuzzySet(NS_eThetaF); // Add FuzzySet 
 FuzzySet* Z_eThetaF = new FuzzySet(-0.5, 0.0, 0.0, 0.5); // 
 eThetaF->addFuzzySet(Z_eThetaF); // Add FuzzySet 
 FuzzySet* PS_eThetaF = new FuzzySet(0.0, 0.5, 0.5, 1.0); // 
 eThetaF->addFuzzySet(PS_eThetaF); // Add FuzzySet 
 FuzzySet* PM_eThetaF = new FuzzySet(0.5, 1.0, 1000, 1000); // 
 eThetaF->addFuzzySet(PM_eThetaF); // Add FuzzySet 
 
 fuzzy->addFuzzyInput(eThetaF); // Add FuzzyInput to Fuzzy object


  FuzzyInput*  deThetaF= new FuzzyInput(2);// With its ID in param
 // Creating the FuzzySet to compond FuzzyInput previousAR
 
 // Creating the FuzzySet to compond FuzzyInput E
 FuzzySet* NM_deThetaF = new FuzzySet(-1000, -1000, -1.0, -0.5); // 
 deThetaF->addFuzzySet(NM_deThetaF); // Add FuzzySet 
 FuzzySet* NS_deThetaF = new FuzzySet(-1.0, -0.5, -0.5, 0.0); // 
 deThetaF->addFuzzySet(NS_deThetaF); // Add FuzzySet 
 FuzzySet* Z_deThetaF = new FuzzySet(-0.5, 0.0, 0.0, 0.5); // 
 deThetaF->addFuzzySet(Z_deThetaF); // Add FuzzySet 
 FuzzySet* PS_deThetaF = new FuzzySet(0.0, 0.5, 0.5, 1.0); // 
 deThetaF->addFuzzySet(PS_deThetaF); // Add FuzzySet 
 FuzzySet* PM_deThetaF = new FuzzySet(0.5, 1.0, 1000, 1000); // 
 deThetaF->addFuzzySet(PM_deThetaF); // Add FuzzySet 

 
 fuzzy->addFuzzyInput(deThetaF); // Add FuzzyInput to Fuzzy object


  // Passo 3 - Creating FuzzyOutput AR
 FuzzyOutput* omegaF = new FuzzyOutput(3);// With its ID in param

 FuzzySet* NM_omegaF = new FuzzySet(-1.0, -1.0, -1.0, -0.5); // 
 omegaF->addFuzzySet(NM_omegaF); // Add FuzzySet 
 FuzzySet* NS_omegaF = new FuzzySet(-1.0, -0.5, -0.5, 0.0); // 
 omegaF->addFuzzySet(NS_omegaF); // Add FuzzySet 
 FuzzySet* Z_omegaF = new FuzzySet(-0.5, 0.0, 0.0, 0.5); // 
 omegaF->addFuzzySet(Z_omegaF); // Add FuzzySet 
 FuzzySet* PS_omegaF = new FuzzySet(0.0, 0.5, 0.5, 1.0); // 
 omegaF->addFuzzySet(PS_omegaF); // Add FuzzySet 
 FuzzySet* PM_omegaF = new FuzzySet(0.5, 1.0, 1.0, 1.0); // 
 omegaF->addFuzzySet(PM_omegaF); // Add FuzzySet 

 fuzzy->addFuzzyOutput(omegaF); // Add FuzzyOutput to Fuzzy object


  // fuzzy rules
 
 FuzzyRuleAntecedent* R1A = new FuzzyRuleAntecedent();
 R1A->joinWithAND(NM_eThetaF, NM_deThetaF);
 FuzzyRuleAntecedent* R2A = new FuzzyRuleAntecedent();
 R2A->joinWithAND(NM_eThetaF, NS_deThetaF);
 FuzzyRuleAntecedent* R3A = new FuzzyRuleAntecedent();
 R3A->joinWithAND(NM_eThetaF, Z_deThetaF);
 FuzzyRuleAntecedent* R4A = new FuzzyRuleAntecedent();
 R4A->joinWithAND(NM_eThetaF, PS_deThetaF);
 FuzzyRuleAntecedent* R5A = new FuzzyRuleAntecedent();
 R5A->joinWithAND(NM_eThetaF, PM_deThetaF);

 FuzzyRuleAntecedent* R6A = new FuzzyRuleAntecedent();
 R6A->joinWithAND(NS_eThetaF, NM_deThetaF);
 FuzzyRuleAntecedent* R7A = new FuzzyRuleAntecedent();
 R7A->joinWithAND(NS_eThetaF, NS_deThetaF);
 FuzzyRuleAntecedent* R8A = new FuzzyRuleAntecedent();
 R8A->joinWithAND(NS_eThetaF, Z_deThetaF);
 FuzzyRuleAntecedent* R9A = new FuzzyRuleAntecedent();
 R9A->joinWithAND(NS_eThetaF, PS_deThetaF);
 FuzzyRuleAntecedent* R10A = new FuzzyRuleAntecedent();
 R10A->joinWithAND(NS_eThetaF, PM_deThetaF);

 FuzzyRuleAntecedent* R11A = new FuzzyRuleAntecedent();
 R11A->joinWithAND(Z_eThetaF, NM_deThetaF);
 FuzzyRuleAntecedent* R12A = new FuzzyRuleAntecedent();
 R12A->joinWithAND(Z_eThetaF, NS_deThetaF);
 FuzzyRuleAntecedent* R13A = new FuzzyRuleAntecedent();
 R13A->joinWithAND(Z_eThetaF, Z_deThetaF);
 FuzzyRuleAntecedent* R14A = new FuzzyRuleAntecedent();
 R14A->joinWithAND(Z_eThetaF, PS_deThetaF);
 FuzzyRuleAntecedent* R15A = new FuzzyRuleAntecedent();
 R15A->joinWithAND(Z_eThetaF, PM_deThetaF);

 FuzzyRuleAntecedent* R16A = new FuzzyRuleAntecedent();
 R16A->joinWithAND(PS_eThetaF, NM_deThetaF);
 FuzzyRuleAntecedent* R17A = new FuzzyRuleAntecedent();
 R17A->joinWithAND(PS_eThetaF, NS_deThetaF);
 FuzzyRuleAntecedent* R18A = new FuzzyRuleAntecedent();
 R18A->joinWithAND(PS_eThetaF, Z_deThetaF);
 FuzzyRuleAntecedent* R19A = new FuzzyRuleAntecedent();
 R19A->joinWithAND(PS_eThetaF, PS_deThetaF);
 FuzzyRuleAntecedent* R20A = new FuzzyRuleAntecedent();
 R20A->joinWithAND(PS_eThetaF, PM_deThetaF);

 FuzzyRuleAntecedent* R21A = new FuzzyRuleAntecedent();
 R21A->joinWithAND(PM_eThetaF, NM_deThetaF);
 FuzzyRuleAntecedent* R22A = new FuzzyRuleAntecedent();
 R22A->joinWithAND(PM_eThetaF, NS_deThetaF);
 FuzzyRuleAntecedent* R23A = new FuzzyRuleAntecedent();
 R23A->joinWithAND(PM_eThetaF, Z_deThetaF);
 FuzzyRuleAntecedent* R24A = new FuzzyRuleAntecedent();
 R24A->joinWithAND(PM_eThetaF, PS_deThetaF);
 FuzzyRuleAntecedent* R25A = new FuzzyRuleAntecedent();
 R25A->joinWithAND(PM_eThetaF, PM_deThetaF);

 FuzzyRuleConsequent* NM = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 NM->addOutput(NM_omegaF);
 FuzzyRuleConsequent* NS = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 NS->addOutput(NS_omegaF);
 FuzzyRuleConsequent* Z = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 Z->addOutput(Z_omegaF);
 FuzzyRuleConsequent* PS = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 PS->addOutput(PS_omegaF);
 FuzzyRuleConsequent* PM = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 PM->addOutput(PM_omegaF);

 // Instantiating a FuzzyRule object
 FuzzyRule* R1 = new FuzzyRule(1, R1A, NM); // Passing the Antecedent and the Consequent of expression
 fuzzy->addFuzzyRule(R1); // Adding FuzzyRule to Fuzzy object
 FuzzyRule* R2 = new FuzzyRule(2, R2A, NM); 
 fuzzy->addFuzzyRule(R2); 
 FuzzyRule* R3 = new FuzzyRule(3, R3A, NM); 
 fuzzy->addFuzzyRule(R3); 
 FuzzyRule* R4 = new FuzzyRule(4, R4A, NS); 
 fuzzy->addFuzzyRule(R4); 
 FuzzyRule* R5 = new FuzzyRule(5, R5A, Z); 
 fuzzy->addFuzzyRule(R5); 
 
 FuzzyRule* R6 = new FuzzyRule(6, R6A, NM); 
 fuzzy->addFuzzyRule(R6); 
 FuzzyRule* R7 = new FuzzyRule(7, R7A, NM); 
 fuzzy->addFuzzyRule(R7); 
 FuzzyRule* R8 = new FuzzyRule(8, R8A, NS); 
 fuzzy->addFuzzyRule(R8); 
 FuzzyRule* R9 = new FuzzyRule(9, R9A, Z); 
 fuzzy->addFuzzyRule(R9);
 FuzzyRule* R10 = new FuzzyRule(10, R10A, PS); 
 fuzzy->addFuzzyRule(R10);

 FuzzyRule* R11 = new FuzzyRule(11, R11A, NM); 
 fuzzy->addFuzzyRule(R11); 
 FuzzyRule* R12 = new FuzzyRule(12, R12A, NS); 
 fuzzy->addFuzzyRule(R12); 
 FuzzyRule* R13 = new FuzzyRule(13, R13A, Z); 
 fuzzy->addFuzzyRule(R13); 
 FuzzyRule* R14 = new FuzzyRule(14, R14A, PS); 
 fuzzy->addFuzzyRule(R14);
 FuzzyRule* R15 = new FuzzyRule(15, R15A, PM); 
 fuzzy->addFuzzyRule(R15);


 FuzzyRule* R16 = new FuzzyRule(16, R16A, NS); 
 fuzzy->addFuzzyRule(R16); 
 FuzzyRule* R17 = new FuzzyRule(17, R17A, Z); 
 fuzzy->addFuzzyRule(R17); 
 FuzzyRule* R18 = new FuzzyRule(18, R18A, PS); 
 fuzzy->addFuzzyRule(R18); 
 FuzzyRule* R19 = new FuzzyRule(19, R19A, PM); 
 fuzzy->addFuzzyRule(R19);
 FuzzyRule* R20 = new FuzzyRule(20, R20A, PM); 
 fuzzy->addFuzzyRule(R20);

 FuzzyRule* R21 = new FuzzyRule(21, R21A, Z); 
 fuzzy->addFuzzyRule(R21); 
 FuzzyRule* R22 = new FuzzyRule(22, R22A, PS); 
 fuzzy->addFuzzyRule(R22); 
 FuzzyRule* R23 = new FuzzyRule(23, R23A, PM); 
 fuzzy->addFuzzyRule(R23); 
 FuzzyRule* R24 = new FuzzyRule(24, R24A, PM); 
 fuzzy->addFuzzyRule(R24);
 FuzzyRule* R25 = new FuzzyRule(25, R25A, PM); 
 fuzzy->addFuzzyRule(R25);

 /////////////////Distance/////////////////////////////////////

// Step 2 - Creating a FuzzyInput E
 FuzzyInput* eDisF = new FuzzyInput(1);// With its ID in param
 // Creating the FuzzySet to compond FuzzyInput E
 FuzzySet* NM_eDisF = new FuzzySet(-1000, -1000, -1.0, -0.5); // 
 eDisF->addFuzzySet(NM_eDisF); // Add FuzzySet 
 FuzzySet* NS_eDisF = new FuzzySet(-1.0, -0.5, -0.5, 0.0); // 
 eDisF->addFuzzySet(NS_eDisF); // Add FuzzySet 
 FuzzySet* Z_eDisF = new FuzzySet(-0.5, 0.0, 0.0, 0.5); // 
 eDisF->addFuzzySet(Z_eDisF); // Add FuzzySet 
 FuzzySet* PS_eDisF = new FuzzySet(0.0, 0.5, 0.5, 1.0); // 
 eDisF->addFuzzySet(PS_eDisF); // Add FuzzySet 
 FuzzySet* PM_eDisF = new FuzzySet(0.5, 1.0, 1000, 1000); // 
 eDisF->addFuzzySet(PM_eDisF); // Add FuzzySet 
 
 fuzzyDis->addFuzzyInput(eDisF); // Add FuzzyInput to Fuzzy object


  FuzzyInput*  deDisF= new FuzzyInput(2);// With its ID in param
 // Creating the FuzzySet to compond FuzzyInput DE
 FuzzySet* NM_deDisF = new FuzzySet(-1000, -1000, -1.0, -0.5); // 
 deDisF->addFuzzySet(NM_deDisF); // Add FuzzySet 
 FuzzySet* NS_deDisF = new FuzzySet(-1.0, -0.5, -0.5, 0.0); // 
 deDisF->addFuzzySet(NS_deDisF); // Add FuzzySet 
 FuzzySet* Z_deDisF = new FuzzySet(-0.5, 0.0, 0.0, 0.5); // 
 deDisF->addFuzzySet(Z_deDisF); // Add FuzzySet 
 FuzzySet* PS_deDisF = new FuzzySet(0.0, 0.5, 0.5, 1.0); // 
 deDisF->addFuzzySet(PS_deDisF); // Add FuzzySet 
 FuzzySet* PM_deDisF = new FuzzySet(0.5, 1.0, 1000, 1000); // 
 deDisF->addFuzzySet(PM_deDisF); // Add FuzzySet 

 
 fuzzyDis->addFuzzyInput(deDisF); // Add FuzzyInput to Fuzzy object


  // Passo 3 - Creating FuzzyOutput AR
 FuzzyOutput* VyF = new FuzzyOutput(3);// With its ID in param

 FuzzySet* NM_VyF = new FuzzySet(-1.0, -1.0, -1.0, -0.5); // 
 VyF->addFuzzySet(NM_VyF); // Add FuzzySet 
 FuzzySet* NS_VyF = new FuzzySet(-1.0, -0.5, -0.5, 0.0); // 
 VyF->addFuzzySet(NS_VyF); // Add FuzzySet 
 FuzzySet* Z_VyF = new FuzzySet(-0.5, 0.0, 0.0, 0.5); // 
 VyF->addFuzzySet(Z_VyF); // Add FuzzySet 
 FuzzySet* PS_VyF = new FuzzySet(0.0, 0.5, 0.5, 1.0); // 
 VyF->addFuzzySet(PS_VyF); // Add FuzzySet 
 FuzzySet* PM_VyF = new FuzzySet(0.5, 1.0, 1.0, 1.0); // 
 VyF->addFuzzySet(PM_VyF); // Add FuzzySet 

 fuzzyDis->addFuzzyOutput(VyF); // Add FuzzyOutput to Fuzzy object


  // fuzzy rules
 
 FuzzyRuleAntecedent* R1B = new FuzzyRuleAntecedent();
 R1B->joinWithAND(NM_eDisF, NM_deDisF);
 FuzzyRuleAntecedent* R2B = new FuzzyRuleAntecedent();
 R2B->joinWithAND(NM_eDisF, NS_deDisF);
 FuzzyRuleAntecedent* R3B = new FuzzyRuleAntecedent();
 R3B->joinWithAND(NM_eDisF, Z_deDisF);
 FuzzyRuleAntecedent* R4B = new FuzzyRuleAntecedent();
 R4B->joinWithAND(NM_eDisF, PS_deDisF);
 FuzzyRuleAntecedent* R5B = new FuzzyRuleAntecedent();
 R5B->joinWithAND(NM_eDisF, PM_deDisF);

 FuzzyRuleAntecedent* R6B = new FuzzyRuleAntecedent();
 R6B->joinWithAND(NS_eDisF, NM_deDisF);
 FuzzyRuleAntecedent* R7B = new FuzzyRuleAntecedent();
 R7B->joinWithAND(NS_eDisF, NS_deDisF);
 FuzzyRuleAntecedent* R8B = new FuzzyRuleAntecedent();
 R8B->joinWithAND(NS_eDisF, Z_deDisF);
 FuzzyRuleAntecedent* R9B = new FuzzyRuleAntecedent();
 R9B->joinWithAND(NS_eDisF, PS_deDisF);
 FuzzyRuleAntecedent* R10B = new FuzzyRuleAntecedent();
 R10B->joinWithAND(NS_eDisF, PM_deDisF);

 FuzzyRuleAntecedent* R11B = new FuzzyRuleAntecedent();
 R11B->joinWithAND(Z_eDisF, NM_deDisF);
 FuzzyRuleAntecedent* R12B = new FuzzyRuleAntecedent();
 R12B->joinWithAND(Z_eDisF, NS_deDisF);
 FuzzyRuleAntecedent* R13B = new FuzzyRuleAntecedent();
 R13B->joinWithAND(Z_eDisF, Z_deDisF);
 FuzzyRuleAntecedent* R14B = new FuzzyRuleAntecedent();
 R14B->joinWithAND(Z_eDisF, PS_deDisF);
 FuzzyRuleAntecedent* R15B = new FuzzyRuleAntecedent();
 R15B->joinWithAND(Z_eDisF, PM_deDisF);

 FuzzyRuleAntecedent* R16B = new FuzzyRuleAntecedent();
 R16B->joinWithAND(PS_eDisF, NM_deDisF);
 FuzzyRuleAntecedent* R17B = new FuzzyRuleAntecedent();
 R17B->joinWithAND(PS_eDisF, NS_deDisF);
 FuzzyRuleAntecedent* R18B = new FuzzyRuleAntecedent();
 R18B->joinWithAND(PS_eDisF, Z_deDisF);
 FuzzyRuleAntecedent* R19B = new FuzzyRuleAntecedent();
 R19B->joinWithAND(PS_eDisF, PS_deDisF);
 FuzzyRuleAntecedent* R20B = new FuzzyRuleAntecedent();
 R20B->joinWithAND(PS_eDisF, PM_deDisF);

 FuzzyRuleAntecedent* R21B = new FuzzyRuleAntecedent();
 R21B->joinWithAND(PM_eDisF, NM_deDisF);
 FuzzyRuleAntecedent* R22B = new FuzzyRuleAntecedent();
 R22B->joinWithAND(PM_eDisF, NS_deDisF);
 FuzzyRuleAntecedent* R23B = new FuzzyRuleAntecedent();
 R23B->joinWithAND(PM_eDisF, Z_deDisF);
 FuzzyRuleAntecedent* R24B = new FuzzyRuleAntecedent();
 R24B->joinWithAND(PM_eDisF, PS_deDisF);
 FuzzyRuleAntecedent* R25B = new FuzzyRuleAntecedent();
 R25B->joinWithAND(PM_eDisF, PM_deDisF);

 FuzzyRuleConsequent* NM2 = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 NM2->addOutput(NM_VyF);
 FuzzyRuleConsequent* NS2 = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 NS2->addOutput(NS_VyF);
 FuzzyRuleConsequent* Z2 = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 Z2->addOutput(Z_VyF);
 FuzzyRuleConsequent* PS2 = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 PS2->addOutput(PS_VyF);
 FuzzyRuleConsequent* PM2 = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
 PM2->addOutput(PM_VyF);

 // Instantiating a FuzzyRule object
 FuzzyRule* R1_2 = new FuzzyRule(1, R1B, NM2); // Passing the Antecedent and the Consequent of expression
 fuzzyDis->addFuzzyRule(R1_2); // Adding FuzzyRule to Fuzzy object
 FuzzyRule* R2_2 = new FuzzyRule(2, R2B, NM2); 
 fuzzyDis->addFuzzyRule(R2_2); 
 FuzzyRule* R3_2 = new FuzzyRule(3, R3B, NM2); 
 fuzzyDis->addFuzzyRule(R3_2); 
 FuzzyRule* R4_2 = new FuzzyRule(4, R4B, NS2); 
 fuzzyDis->addFuzzyRule(R4_2); 
 FuzzyRule* R5_2 = new FuzzyRule(5, R5B, Z2); 
 fuzzyDis->addFuzzyRule(R5_2); 
 
 FuzzyRule* R6_2 = new FuzzyRule(6, R6B, NM2); 
 fuzzyDis->addFuzzyRule(R6_2); 
 FuzzyRule* R7_2 = new FuzzyRule(7, R7B, NM2); 
 fuzzyDis->addFuzzyRule(R7_2); 
 FuzzyRule* R8_2 = new FuzzyRule(8, R8B, NS2); 
 fuzzyDis->addFuzzyRule(R8_2); 
 FuzzyRule* R9_2 = new FuzzyRule(9, R9B, Z2); 
 fuzzyDis->addFuzzyRule(R9_2);
 FuzzyRule* R10_2 = new FuzzyRule(10, R10B, PS2); 
 fuzzyDis->addFuzzyRule(R10_2);

 FuzzyRule* R11_2 = new FuzzyRule(11, R11B, NM2); 
 fuzzyDis->addFuzzyRule(R11_2); 
 FuzzyRule* R12_2 = new FuzzyRule(12, R12B, NS2); 
 fuzzyDis->addFuzzyRule(R12_2); 
 FuzzyRule* R13_2 = new FuzzyRule(13, R13B, Z2); 
 fuzzyDis->addFuzzyRule(R13_2); 
 FuzzyRule* R14_2 = new FuzzyRule(14, R14B, PS2); 
 fuzzyDis->addFuzzyRule(R14_2);
 FuzzyRule* R15_2 = new FuzzyRule(15, R15B, PM2); 
 fuzzyDis->addFuzzyRule(R15_2);


 FuzzyRule* R16_2 = new FuzzyRule(16, R16B, NS2); 
 fuzzyDis->addFuzzyRule(R16_2); 
 FuzzyRule* R17_2 = new FuzzyRule(17, R17B, Z2); 
 fuzzyDis->addFuzzyRule(R17_2); 
 FuzzyRule* R18_2 = new FuzzyRule(18, R18B, PS2); 
 fuzzyDis->addFuzzyRule(R18_2); 
 FuzzyRule* R19_2 = new FuzzyRule(19, R19B, PM2); 
 fuzzyDis->addFuzzyRule(R19_2);
 FuzzyRule* R20_2 = new FuzzyRule(20, R20B, PM2); 
 fuzzyDis->addFuzzyRule(R20_2);

 FuzzyRule* R21_2 = new FuzzyRule(21, R21B, Z2); 
 fuzzyDis->addFuzzyRule(R21_2); 
 FuzzyRule* R22_2 = new FuzzyRule(22, R22B, PS2); 
 fuzzyDis->addFuzzyRule(R22_2); 
 FuzzyRule* R23_2 = new FuzzyRule(23, R23B, PM2); 
 fuzzyDis->addFuzzyRule(R23_2); 
 FuzzyRule* R24_2 = new FuzzyRule(24, R24B, PM2); 
 fuzzyDis->addFuzzyRule(R24_2);
 FuzzyRule* R25_2 = new FuzzyRule(25, R25B, PM2); 
 fuzzyDis->addFuzzyRule(R25_2);


 /////////////////////////////////////////////////////////
}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  //Serial3.print("etheta ");
  //Serial3.print(angle, 4);
  //Serial3.print(", ");
  //Serial3.print("y1 ");
  //Serial3.print(y1);
  //Serial3.print(", ");
  //Serial3.print("Fsv");
  //Serial3.println(Fsv);
  
  enableMuxPort(4);
  y1 = sensor.getDistance();
  disableMuxPort(4);
  delay(1);
  enableMuxPort(5);
  y2 = sensor.getDistance();
  disableMuxPort(5);
  delay(1);
  
  angle  = (atan((y2-y1)/(x2-x1)))*(180/3.14);
  
  angle_2 = step_theta*(180/3.14);
  
  if(event.orientation.x>=180.0)
  {
    eTheta = (event.orientation.x-360);
  }
  else
  {
  eTheta = event.orientation.x;
  }
  
  if(long_step_detect == 0)
  {
    deTheta= angle-eThetaP;
    eThetaP = angle;
  }
  
  else
  {
    deTheta= angle_2-eThetaP;
    eThetaP = angle_2;
    angle=angle_2;
  }

  int dis, disRef;

  enableMuxPort(3);
  dis = sensor.getDistance();
  disableMuxPort(3);
  Serial3.println(dis);
  disRef = 45;//25
  eDis = dis-disRef; // dis need to take from perception module
  deDis = eDis-eDisP;
  eDisP = eDis;

//  if((step_x!=0) or (step_y !=0) or (step_theta !=0))
//  {
//    float v_lx= step_x;
//    float v_ly= step_y;
//    float v_w= step_theta;
//
//    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
//    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
//    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
//    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
//  
//    roboclaw.SpeedM1(address1, int(vel_fr*4480));
//    roboclaw.SpeedM2(address1, int(vel_fl*4480));
//    roboclaw.SpeedM1(address2, int(vel_br*4480));
//    roboclaw.SpeedM2(address2, int(vel_bl*4480));
//    cmd_vel_flag = 1;
//  }
//  else
//  {
//    cmd_vel_flag = 0;
//  }
  
  if (Serial3.available() > 0)
    i = Serial3.read();
 
  if (i == 'F')
  {
    if (timer_longstep == 0)
      {
        PrevMillis_3 = millis();
        timer_longstep = 1;
      }

    fuzzy->setInput(1, angle/1.0);
    fuzzy->setInput(2, deTheta);
    fuzzy->fuzzify();
  
  if(long_step_detect == 1)
  {
    float v_lx= 0.025;
    float v_ly= 0.0;
    float v_w= omega*0.30;
  }

  else
  {
    float v_lx= 0.025;
    float v_ly= 0.0;
    float v_w= omega*0.020;
  }

  
    omega = fuzzy->defuzzify(3);

    
    float v_lx= 0.025;
    float v_ly= 0.0;
    float v_w= omega*0.020;

    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));

    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3,2000);
    roboclaw2.SpeedM2(address3,2000);
    
    enableMuxPort(0);
    Fsv = sensor.getDistance();
    disableMuxPort(0);
    delay(1);
    enableMuxPort(1);
    Msv = sensor.getDistance();
    disableMuxPort(1);
    delay(1);
    enableMuxPort(2);
    Bsv = sensor.getDistance();
    disableMuxPort(2);
    delay(1);
    Serial3.print("hello");
    Serial3.println(Fsv);
    if ((Fsv > 100) or (Msv > 80) or (Bsv > 100))
    {
      if (timer_flag == 0)
      {
        prevMillis_2 = millis();
        timer_flag = 1;
      }
      if (((millis() - prevMillis_2) > 45))
      {
        if((Fsv > 100) and (Msv < 40) and (Bsv < 40))
        {
          roboclaw.SpeedM1(address1, 0);
          roboclaw.SpeedM2(address1, 0);
          roboclaw.SpeedM1(address2, 0);
          roboclaw.SpeedM2(address2, 0);
          roboclaw2.SpeedM1(address3,0);
          roboclaw2.SpeedM2(address3,0);
          i = 'D';
          f_state = 1;
          timer_flag = 0;
        }
        else if((Fsv < 40) and (Msv > 80) and (Bsv < 40)and ((millis() - prevMillis_2) > 250))
        {
          roboclaw.SpeedM1(address1, 0);
          roboclaw.SpeedM2(address1, 0);
          roboclaw.SpeedM1(address2, 0);
          roboclaw.SpeedM2(address2, 0);
          roboclaw2.SpeedM1(address3,0);
          roboclaw2.SpeedM2(address3,0);
          i = 'M';
          f_state = 2;
          timer_flag = 0;
        }
        else if((Fsv < 40) and (Msv < 40) and (Bsv >100) and ((millis() - prevMillis_2) > 100))
        {
          roboclaw.SpeedM1(address1, 0);
          roboclaw.SpeedM2(address1, 0);
          roboclaw.SpeedM1(address2, 0);
          roboclaw.SpeedM2(address2, 0);
          roboclaw2.SpeedM1(address3,0);
          roboclaw2.SpeedM2(address3,0);
          i = 'T';
          f_state=0;
          timer_flag = 0;
        }
      }
      p_i_2 = 0;
      timer_longstep=0;
      long_step_detect = 0;
    }
    if (((millis() - PrevMillis_3) > 25000))
    {
      long_step_detect = 1;
      timer_longstep=0;
      i = 'L';
    }
    p_i = 'F';
  }

  else if (i == 'B')
  {
    if (timer_flag == 0)
      {
        prevMillis_2 = millis();
        timer_flag = 1;
      }
    
    fuzzy->setInput(1, angle/20.0);
    fuzzy->setInput(2, deTheta);
    fuzzy->fuzzify();
  
    omega = fuzzy->defuzzify(3);

    
    float v_lx= -0.030;
    float v_ly= 0.0;
    float v_w= omega*0.020;

    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));

    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, -2000);
    roboclaw2.SpeedM2(address3, -2000);
    
    if(((angle<3 and angle>-3) and (dis<40)) or ((millis() - prevMillis_2) > 3000))
    {
          roboclaw.SpeedM1(address1, 0);
          roboclaw.SpeedM2(address1, 0);
          roboclaw.SpeedM1(address2, 0);
          roboclaw.SpeedM2(address2, 0);
          roboclaw2.SpeedM1(address3,0);
          roboclaw2.SpeedM2(address3,0);  
          timer_flag = 0;
          if(RL_flag==0)
          {
          i = 'R';
          } 
          else if(RL_flag==1)
          {
          i = 'L';
          }
    }
  }

  else if (i == 'R')
  {
    fuzzy->setInput(1, angle/20.0);
    fuzzy->setInput(2, deTheta);
    fuzzy->fuzzify(); 
    omega = fuzzy->defuzzify(3);

    fuzzyDis->setInput(1, eDis/25.0);
    fuzzyDis->setInput(2, deDis);
    fuzzyDis->fuzzify(); 
    Vx = fuzzyDis->defuzzify(3);
    
    float v_lx= -Vx*0.009;
    float v_ly= 0.06;
    float v_w= omega*0.03;

    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));

    //Serial3.println(int(vel_fr*4480));
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, 0);
    roboclaw2.SpeedM2(address3, 0);
    enableMuxPort(6);
    Rsv = sensor.getDistance();
    disableMuxPort(6);
    delay(1);
    if(Rsv < 100 and Rsv > 70)
    {
      timer_flag = 0;
      roboclaw.SpeedM1(address1, 0);
      roboclaw.SpeedM2(address1, 0);
      roboclaw.SpeedM1(address2, 0);
      roboclaw.SpeedM2(address2, 0);
      delay(1);
      f_state = 0;
      RL_flag=1;
      i = 'A';
    }
    p_i = 'R';
    p_i_2 = 1;
  }

  else if (i == 'L')
  {


    fuzzy->setInput(1, angle/20.0);
    fuzzy->setInput(2, deTheta);
    fuzzy->fuzzify(); 
    omega = fuzzy->defuzzify(3);

    fuzzyDis->setInput(1, eDis/25.0);
    fuzzyDis->setInput(2, deDis);
    fuzzyDis->fuzzify(); 
    Vx = fuzzyDis->defuzzify(3);
    
    if(long_step_detect==1 and timer_longstep==0)
    {
      PrevMillis_3 = millis();
      float v_lx= 0.0;
      float v_ly= -0.06;
      float v_w= 0.0;
      timer_longstep = 1;
      float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
      float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
      float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
      float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
    
      roboclaw.SpeedM1(address1, int(vel_fr*4480));
      roboclaw.SpeedM2(address1, int(vel_fl*4480));
      roboclaw.SpeedM1(address2, int(vel_br*4480));
      roboclaw.SpeedM2(address2, int(vel_bl*4480));
      roboclaw2.SpeedM1(address3, 0);
      roboclaw2.SpeedM2(address3, 0);
    }
    
    else if(long_step_detect==0)
    {
      float v_lx= -Vx*0.009;
      float v_ly= -0.06;
      float v_w= omega*0.025;
      float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
      float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
      float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
      float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
    
      roboclaw.SpeedM1(address1, int(vel_fr*4480));
      roboclaw.SpeedM2(address1, int(vel_fl*4480));
      roboclaw.SpeedM1(address2, int(vel_br*4480));
      roboclaw.SpeedM2(address2, int(vel_bl*4480));
      roboclaw2.SpeedM1(address3, 0);
      roboclaw2.SpeedM2(address3, 0);
    }

    delay(5);
    enableMuxPort(7);
    Lsv = sensor.getDistance();
    disableMuxPort(7);
    delay(1);
    
    
    if(Lsv<100 and Lsv>70 and long_step_detect==0)
    {
      roboclaw.SpeedM1(address1, 0);
      roboclaw.SpeedM2(address1, 0);
      roboclaw.SpeedM1(address2, 0);
      roboclaw.SpeedM2(address2, 0);
      delay(1);
      f_state = 0;
      RL_flag=0;
      i = 'A';
    }
    
    if(p_i_2 == 1 and long_step_detect==1)
    {
     if (((millis() - PrevMillis_3) > 26000))
    {
      timer_longstep=0;
      i = 'Z';
    }
    }
    
    else if (p_i_2 == 0 and long_step_detect==1)
    {
     if (((millis() - PrevMillis_3) > 14000))
    {
      timer_longstep=0;
      i = 'Z';
    }
    }
    
    p_i = 'L';
    
  }
  
  else if(i == 'A')
  {
//    float v_lx= 0.0;
//    float v_ly= 0.0;
//    float v_w= -0.05;
//
//    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
//    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
//    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
//    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
//    
//    roboclaw.SpeedM1(address1, int(vel_fr*4480));
//    roboclaw.SpeedM2(address1, int(vel_fl*4480));
//    roboclaw.SpeedM1(address2, int(vel_br*4480));
//    roboclaw.SpeedM2(address2, int(vel_bl*4480));
//    delay(250);
    bno.begin();
//    if(RL_flag==0)
//    {
//     i = 'R';
//    }
//    else if(RL_flag==1)
//    {
//     i = 'L';
//    }
    i = 'F';
  }
  
  else if (i == 'Z')
  {
    fuzzy->setInput(1, angle /20.0);
    fuzzy->setInput(2, deTheta);
    fuzzy->fuzzify(); 
    omega = fuzzy->defuzzify(3);

    float v_lx= 0.0;
    float v_ly= 0.0;
    float v_w= 0.05;

    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, 2000);
    roboclaw2.SpeedM2(address3, -2000);
     
    if((eTheta<-170) and (eTheta>-175))
    {
      i = 'F';
    }
  }

  else if (i == 'Q')
  { 
    float v_lx= 0.0;
    float v_ly= 0.0;
    float v_w= 0.08;

    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
    
    roboclaw.SpeedM1(address1, int(vel_fr*4480));
    roboclaw.SpeedM2(address1, int(vel_fl*4480));
    roboclaw.SpeedM1(address2, int(vel_br*4480));
    roboclaw.SpeedM2(address2, int(vel_bl*4480));
    roboclaw2.SpeedM1(address3, 2000);
    roboclaw2.SpeedM2(address3, -2000);
    
  }

  else if (i == 'E')
  {
    float v_lx= 0.0;
    float v_ly= 0.0;
    float v_w= -0.08;

    float vel_fl =((1/R)*(v_lx-v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_fr =((1/R)*(v_lx+v_ly+(L+B)*v_w))*(60/(3.14*70));
    float vel_bl =((1/R)*(v_lx+v_ly-(L+B)*v_w))*(60/(3.14*70));
    float vel_br =((1/R)*(v_lx-v_ly+(L+B)*v_w))*(60/(3.14*70));
    
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
    disableMuxPort(0);
    disableMuxPort(1);
    disableMuxPort(2);
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

  //stetro_msg.linear.x = step_theta;
  odom();
  //stetro_status_msg.publish( &stetro_msg );
  //nh.spinOnce();
}

void odom()
{
  double vel_dt = (millis() - PrevOdomMillis);
  PrevOdomMillis = millis();
  
  uint8_t status1,status2, status3, status4;
  bool valid1, valid2, valid3, valid4;
  int32_t encFR = roboclaw.ReadSpeedM1(address1, &status1, &valid1);
  int32_t encFL = roboclaw.ReadSpeedM2(address1, &status2, &valid2);
  int32_t encBR = roboclaw.ReadSpeedM1(address2, &status3, &valid3);
  int32_t encBL = roboclaw.ReadSpeedM2(address2, &status4, &valid4);

  float speed_FR = (encFR*60)/4480;
  float speed_FL = (encFL*60)/4480;
  float speed_BR = (encBR*60)/4480;
  float speed_BL = (encBL*60)/4480;

  double average_x_speed = ((speed_FR+speed_FL+speed_BR+speed_BL)*0.25)/60;
  double average_y_speed = ((speed_FR-speed_FL-speed_BR+speed_BL)*0.25)/60;
  double average_anglular_speed = ((speed_FR-speed_FL+speed_BR-speed_BL)*0.25)/60;

  float linear_vel_x = average_x_speed*(0.075*3.14);
  float linear_vel_y = average_y_speed*(0.075*3.14);
  float angular_vel = (average_anglular_speed*(0.075*3.14))/0.58;

  double delta_theta = (angular_vel * vel_dt)/1000;
  double delta_x = ((linear_vel_x * cos(theta) - linear_vel_y * sin(theta))*vel_dt)/1000;
  double delta_y = ((linear_vel_x * sin(theta) + linear_vel_y * cos(theta))*vel_dt)/1000;

  x_pos = delta_x+x_pos;
  y_pos = delta_y+y_pos;
  theta = delta_theta+theta;

  //stetro_msg.linear.y = speed_FR;
  //stetro_msg.angular.x = speed_FL;
  //stetro_msg.angular.y = speed_BR;
  //stetro_msg.angular.z = speed_BL;
  
  //Serial3.println("");
  //Serial3.print("pose: ");
  //Serial3.print(" x_pos, ");
  Serial3.print(x_pos);
  Serial3.print(", ");
  Serial3.print(y_pos);
  Serial3.print(", ");
  Serial3.print(eTheta);
  //Serial3.println("");
}

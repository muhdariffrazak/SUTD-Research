/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*1
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

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

const uint8_t DXL_ID3 = 2;
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 3;

const float DXL_PROTOCOL_VERSION23 = 1.0;
const float DXL_PROTOCOL_VERSION1 = 2.0;

DynamixelShield dx1;
DynamixelShield dx3;
DynamixelShield dx2;
//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dx1.begin(57600);
  dx2.begin(57600);
  dx3.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dx1.setPortProtocolVersion(DXL_PROTOCOL_VERSION1);
  dx2.setPortProtocolVersion(DXL_PROTOCOL_VERSION23);
  dx3.setPortProtocolVersion(DXL_PROTOCOL_VERSION23);


  // Get DYNAMIXEL information
  dx1.ping(DXL_ID1);
  dx2.ping(DXL_ID2);
  dx3.ping(DXL_ID3);

  // Turn off torque when configuring items in EEPROM area
  dx1.torqueOff(DXL_ID1);
  dx1.setOperatingMode(DXL_ID1, OP_POSITION);
  dx1.torqueOn(DXL_ID1);

  dx3.torqueOff(DXL_ID3);
  dx3.setOperatingMode(DXL_ID3, OP_POSITION);
  dx3.torqueOn(DXL_ID3);

  dx2.torqueOff(DXL_ID2);
  dx2.setOperatingMode(DXL_ID2, OP_POSITION);
  dx2.torqueOn(DXL_ID2);
}


void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  MoveDX1();
  delay(500);

  MoveDX2();
  delay(500);

  MoveDX3();
  delay(500);
}

void MoveDX1(){
  dx1.setGoalPosition(DXL_ID1, 512);
  delay(1000);
  // Print present position in raw value
  DEBUG_SERIAL.print("Present Position(raw) for 1 : ");
  DEBUG_SERIAL.println(dx1.getPresentPosition(DXL_ID1));
  delay(1000);

  // Set Goal Position in DEGREE value
  dx1.setGoalPosition(DXL_ID1, 5.7, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  DEBUG_SERIAL.print("Present Position(degree) for 1 : ");
  DEBUG_SERIAL.println(dx1.getPresentPosition(DXL_ID1, UNIT_DEGREE));
  delay(1000);
}

void MoveDX3(){
  dx3.setGoalPosition(DXL_ID3, 512);
  delay(1000);
  // Print present position in raw value
  DEBUG_SERIAL.print("Present Position(raw) for 3 : ");
  DEBUG_SERIAL.println(dx3.getPresentPosition(DXL_ID3));
  delay(1000);

  // Set Goal Position in DEGREE value  dxl.setGoalPosition(DXL_ID3, 5.7, UNIT_DEGREE);
  dx3.setGoalPosition(DXL_ID3, 5.7, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  DEBUG_SERIAL.print("Present Position(degree) for 3 : ");
  DEBUG_SERIAL.println(dx3.getPresentPosition(DXL_ID3, UNIT_DEGREE));
  delay(1000);
}

void MoveDX2(){
  dx2.setGoalPosition(DXL_ID2, 512);
  delay(1000);
  // Print present position in raw value
  DEBUG_SERIAL.print("Present Position(raw) for 2 : ");
  DEBUG_SERIAL.println(dx2.getPresentPosition(DXL_ID2));
  delay(1000);

  // Set Goal Position in DEGREE value  dxl.setGoalPosition(DXL_ID3, 5.7, UNIT_DEGREE);
  dx2.setGoalPosition(DXL_ID2, 5.7, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  DEBUG_SERIAL.print("Present Position(degree) for 2 : ");
  DEBUG_SERIAL.println(dx2.getPresentPosition(DXL_ID2, UNIT_DEGREE));
  delay(1000);
}
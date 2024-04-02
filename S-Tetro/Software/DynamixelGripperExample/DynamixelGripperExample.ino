
#include <DynamixelGripper.h>
#include <DynamixelCleaningMotor.h>

//DynamixelGripper A(1, 57600, 2.0);
DynamixelCleaningMotor B(3 ,57600, 1.0, "R");
static int numOfMotors=0;
//DynamixelGripper C(3,57600,1.0);
void setup() {
  // put your setup code here, to run once:
//A.initGripper();
B.initCleaningMotor();
//C.initGripper();

}

void loop() 
{
  // put your main code here, to run repeatedly:
  //A.SafeStartGripper();
  //delay(1000);
  //A.closeGripper();
  B.cleanMode();
  //C.closeGripper();
  delay(2000);
  //A.openGripper();
  B.climbMode();
  delay(2000);
  //C.openGripper();

}

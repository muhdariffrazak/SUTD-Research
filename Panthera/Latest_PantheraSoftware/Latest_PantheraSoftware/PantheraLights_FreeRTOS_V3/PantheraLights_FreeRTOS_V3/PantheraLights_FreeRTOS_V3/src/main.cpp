//libaries and header files
#include <Arduino_FreeRTOS.h>
#include <FastLED.h>
#include <signallights.h>



//--------------------------------------------------
//object for signal lights
signallights A(50);


//--------------------------------------------------
// To handle task suspension/resumtion.
TaskHandle_t Handle_TaskBrakeLight;
TaskHandle_t Handle_TaskSerialRead;
TaskHandle_t Handle_TaskHeadLight;
TaskHandle_t Handle_TaskSignalLight;
TaskHandle_t Handle_TaskBeacon;


//--------------------------------------------------
// define two tasks for Blink & AnalogRead
void TaskBrakeLight( void *pvParameters );
void TaskSerialRead( void *pvParameters );
void TaskHeadLight( void *pvParameters );
void TaskSignalLight( void *pvParameters );
void TaskBeacon( void *pvParameters );

//--------------------------------------------------
//Flags to trigger tasks 
bool BrakeOn=false;// output as feedback
bool DayLight=false;// output as feedback
bool NightLight=false;// output as feedback                                    

bool Autonomous=false;// output as feedback
bool SignalRight=false;// output as feedback
bool TeleOp=false;// output as feedback
bool SignalLeft=false;// output as feedback
bool HazardLight=false;// output as feedback
bool Beacon=false;// output as feedback

int BrakePin=6;
int DayPin=7;
int NightPin=13;
int BeaconPin=12;

const unsigned int MAX_BUFFER_LENGTH = 64;
const unsigned int MAX_MESSAGE_LENGTH = 11;
static char message[MAX_MESSAGE_LENGTH];
bool finished = false;
static unsigned int message_pos = 0;

//--------------------------------------------------
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }


//--------------------------------------------------
  // Tasks are created here.
  xTaskCreate(
    TaskSerialRead
    ,  "SerialRead"
    ,  128  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  &Handle_TaskSerialRead );

  xTaskCreate(
    TaskBrakeLight
    ,  "BrakeLight"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Handle_TaskBrakeLight );
  xTaskCreate(
    TaskHeadLight
    ,  "HeadLight"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &Handle_TaskHeadLight );
  xTaskCreate(
    TaskSignalLight
    ,  "SignalLight"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &Handle_TaskSignalLight );
  xTaskCreate(
    TaskBeacon
    ,  "Beacon"
    ,  100  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &Handle_TaskBeacon );
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}


//--------------------------------------------------
void loop()
{
  // Empty. Things are done in Tasks.
}


//--------------------------------------------------
//Task definitions


//--------------------------------------------------
//Task for serial read
void TaskSerialRead(void *pvParameters)  // Task to serial read and set flags
{
  (void) pvParameters;

  for (;;)//Task loop
  {
    if(Serial.available()>0){
   //Create a place to hold the incoming message
   //Read the next available byte in the serial receive buffer
   //if (Serial.find('*')){
      char inByte = Serial.read();

      //Message coming in (check not terminating character) and guard for over message size
      if((inByte != '\n'&& inByte != '\r') && (message_pos <= MAX_MESSAGE_LENGTH-1) )
      {
        //Add the incoming byte to our message
        if (message_pos==10){
          message[message_pos]='*';
          finished=true;
          //Serial.println(message);
        }
        else{
          message[message_pos] = inByte;
          message_pos++;
        }
      }
   }

   if (finished){
      finished=false;

      //This code parses the message and sets the flags
      if ((message[0]=='*')&&(message[10]=='*')){


        //--------Brake light switch on/off--------
        if (message[1]=='1'){
          //Serial.println("Serial BrakeOn");
          BrakeOn=true;
          vTaskResume(Handle_TaskBrakeLight);
        }
        if (message[1]=='0'){   //CHANGE TO ELSE STATEMENT
          //Serial.println("Serial BrakeOff");        
          BrakeOn=false;
          vTaskResume(Handle_TaskBrakeLight);
        }
        //---------------------------------------


        //--------Hazard light switch on/off-----
        if (message[2]=='1'){   //CHAGE TO MESSAGE[PRIORITY]
          HazardLight=true;
          message[3]='0'; //CHANGE THE PRIORITY
          message[4]='0'; //CHANGE THE PRIORITY
          message[5]='0'; //CHANGE THE PRIORITY
          message[6]='0'; //CHANGE THE PRIORITY
          vTaskResume(Handle_TaskSignalLight);
        }    
        else{   //CHANGE TO ELSE STATEMENT  
          HazardLight=false;
          vTaskResume(Handle_TaskSignalLight);
        }
        //---------------------------------------


        //-------Right Signal(Hard)----------
        if (message[3]=='1'){
          message[4]='0'; //CHANGE THE PRIORITY
          message[5]='0'; //CHANGE THE PRIORITY
          message[6]='0'; //CHANGE THE PRIORITY
          SignalRight=true;
          vTaskResume(Handle_TaskSignalLight);
        }
        else{
          SignalRight=false;
          vTaskResume(Handle_TaskSignalLight);
          }
        //---------------------------------------


        //-------Left Signal(Hard)--------------
        if (message[4]=='1'){
          SignalLeft=true;
          message[3]='0'; //CHANGE THE PRIORITY
          message[5]='0'; //CHANGE THE PRIORITY
          message[6]='0'; //CHANGE THE PRIORITY
          SignalLeft=true;
          vTaskResume(Handle_TaskSignalLight);
          
        }
        else{
          SignalLeft=false;
          vTaskResume(Handle_TaskSignalLight);
          }
        //---------------------------------------


        //-------Autonomous----------------------
        if (message[5]=='1'){
          message[6]='0'; //CHANGE THE PRIORITY
          Autonomous=true;
          vTaskResume(Handle_TaskSignalLight);
        }
        else{
          Autonomous=false;
          vTaskResume(Handle_TaskSignalLight);
        }
        //---------------------------------------


        //--------TeleOp-------------------------
        if (message[6]=='1'){
          message[5]='0'; //CHANGE THE PRIORITY 
          TeleOp=true;
          vTaskResume(Handle_TaskSignalLight);
        }
        else{
          TeleOp=false;
          vTaskResume(Handle_TaskSignalLight);
        }
        //---------------------------------------


        //-------Beacon Light--------------------
        if (message[7]=='1'){  
          Beacon=true;
          vTaskResume(Handle_TaskBeacon);
        }  

        else{   //CHANGE TO ELSE STATEMENT  
          Beacon=false;
          vTaskResume(Handle_TaskBeacon) ;
        }
        //---------------------------------------

        //--------Head light(Night)---------
        if (message[8]=='1'){
          NightLight=true;
          message[9]='0'; //CHANGE THE PRIORITY
          vTaskResume(Handle_TaskHeadLight);
        }
        else{
          if (message[9]=='0'){
            NightLight=false;
            DayLight=false;
            vTaskResume(Handle_TaskHeadLight);
          }
          NightLight=false;
          vTaskResume(Handle_TaskHeadLight);
        }
        //-----------------------------------


        //--------Head light(Day)----------------
        if (message[9]=='1'){
          DayLight=true;
          vTaskResume(Handle_TaskHeadLight);
        }
        else{
          if (message[8]=='0'){
            NightLight=false;
            DayLight=false;
            vTaskResume(Handle_TaskHeadLight);
          }
          DayLight=false;
          vTaskResume(Handle_TaskHeadLight);
        }
        //-----------------------------------

        //Smessage = convertToString(message);
        Serial.println(message);
      }
      else{
        Serial.println("error");
        Serial.end();
        Serial.begin(115200);
      }
      message_pos=0;
      Serial.flush();
      vTaskDelay(50/portTICK_PERIOD_MS);
    }
  }
}


//--------------------------------------------------
//Task for brake light
void TaskBrakeLight(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  pinMode(BrakePin, OUTPUT);
  analogWrite(BrakePin,0);    // turn the LED off by making the voltage LOW

  for (;;) // A Task shall never return or exit.
  {
    if(BrakeOn==true){
    analogWrite(BrakePin,255);   // turn the LED on (HIGH is the voltage level)
    }
    else{
    analogWrite(BrakePin,0);    // turn the LED off by making the voltage LOW
    }
    vTaskSuspend(NULL);
  }
}


//--------------------------------------------------
//Task for head light
void TaskHeadLight(void *pvParameters)  // Task to control headlight
{
  (void) pvParameters;

  pinMode(DayPin, OUTPUT);
  pinMode(NightPin, OUTPUT);
  analogWrite(DayPin,0);    // turn the LED off by making the voltage LOW
  analogWrite(NightPin,0);  
  
  for (;;) // A Task shall never return or exit.
  {
    if(DayLight==true){
      analogWrite(DayPin,255);   // turn the LED on (HIGH is the voltage level)
    }

    else if(NightLight==true){
      analogWrite(NightPin,255);   // turn the LED on (HIGH is the voltage level
    }
    else{
      analogWrite(NightPin,0); 
      analogWrite(DayPin,0); // turn the LED off by making the voltage LOW
      NightLight=false;
      DayLight=false;
    }
    vTaskSuspend(NULL);
  }
}


//--------------------------------------------------
//Task for signal light
void TaskSignalLight(void *pvParameters)  // Task to control signal light
{
  (void) pvParameters;
  A.init_signallights();

  for (;;)//Loop for the signal light
  {
    if(HazardLight==true){
      A.HazardLightOn();
      HazardLight=false;
    }
    
    else //if ((SignalRight==false)&&(SignalLeft==false))
    {
      if (Autonomous==true){
        A.Autonomous();
        Autonomous=false;
      }
      else if (TeleOp==true){
        A.TeleOp();
        TeleOp=false;
      }
    }

    // else if (SignalRight==true){
    //   A.RightHardSignalOn();
    //   if (Autonomous==true){
    //     A.AutonomousTurnRight();
    //     Autonomous=false;
    //   }
    //   else if (TeleOp==true){
    //     A.TeleOpTurnRight();
    //     TeleOp=false;
    //   }
    //   SignalRight=false;
    // }

    // else if (SignalLeft==true){
    //   A.LeftHardSignalOn();
    //   if (Autonomous==true){
    //     A.AutonomousTurnLeft();
    //     Autonomous=false;
    //   }
    //   else if (TeleOp==true){
    //     A.TeleOpTurnLeft();
    //     TeleOp=false;
    //   }
    //   SignalLeft=false;
    // }
    vTaskSuspend(NULL);
  }
}


void TaskBeacon(void *pvParameters)  // Task to control hazard light
{
  (void) pvParameters;


  for (;;)//Loop for the hazard light
  {
    if (Beacon==true){
      analogWrite(BeaconPin,255);
    }
    else{
      analogWrite(BeaconPin,0);
      
    }
    vTaskSuspend(NULL);

  }
}






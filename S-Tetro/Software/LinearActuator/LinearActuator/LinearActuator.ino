// Motor A connections
//int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
/*int enB = 3;
int in3 = 5;
int in4 = 4;*/

void setup() {
	// Set all the motor control pins to outputs
//	pinMode(enA, OUTPUT);
	//pinMode(enB, OUTPUT);
  Serial.begin(9600);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	//pinMode(in3, OUTPUT);
//	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	//digitalWrite(in3, LOW);
	//digitalWrite(in4, LOW);
}

void loop() {
  int sensorValue = analogRead(A0);
  Serial.println(sensorValue);
  Extend();
  Contract();


	// Turn on motors
	// Accelerate from zero to maximum speed
	
}

void Extend(){
  int sensorValue = analogRead(A0);
  if (sensorValue >= 1000){
    Serial.println("Already extended");
  }
  else{
    while (sensorValue < 1000){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      sensorValue = analogRead(A0);
      Serial.println(sensorValue);
    }
    digitalWrite(in1, LOW);
	  digitalWrite(in2, LOW);
  }
}

void Contract(){
  int sensorValue = analogRead(A0);
  if (sensorValue <= 15){
    Serial.println("Already contracted");
  }
  else{
    while (sensorValue >= 15){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      sensorValue = analogRead(A0);
      Serial.println(sensorValue);
    }
    digitalWrite(in1, LOW);
	  digitalWrite(in2, LOW);
  }
}
// This function lets you control speed of the motor
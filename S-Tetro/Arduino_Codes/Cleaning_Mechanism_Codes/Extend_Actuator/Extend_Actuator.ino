int lin = 6;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

void setup() {
  pinMode(lin,OUTPUT); // declare pwm pin to be an output:
}

void loop() {
  analogWrite(lin, 255); // set the brightness of led

}
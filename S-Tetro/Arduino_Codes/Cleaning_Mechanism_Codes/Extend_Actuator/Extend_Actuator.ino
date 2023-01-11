const int Act_PIN = 2;

const int Shaft_max = 254;
const int Shaft_min = 1;

void setup(){
    pinMode( Act_PIN, OUTPUT );
}

void loop(){
    analogWrite( Act_PIN, Shaft_max );
    delay( 10000 );
    
    analogWrite( Act_PIN, Shaft_min );
    delay( 10000 );
}
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  pinMode(0, OUTPUT);   // Pin 0 maps to the USER LED on the ArbotiX Robocontrol
  Serial.begin(9600);
}

char LED = 0;

void loop() {
    if (Serial.available()> 0){
        LED = Serial.read();
        Serial.print(LED);
    }

    if (LED == '3') {
        digitalWrite(0, HIGH);
        delay(1000);
        digitalWrite(0, LOW);
        delay(1000);
    }
    else if (LED == '1') 
        digitalWrite(0, HIGH);
    else if (LED == '0') 
        digitalWrite(0, LOW);
}

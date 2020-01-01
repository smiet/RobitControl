#include <ax12.h>
#include <BioloidController.h>


void setup() {
  Serial.begin(38400);
  Serial.println("serial delimit test 1.0"); // so I can keep track of what is loaded
}
/*
void loop() {
  for (int id = 0; id <= 16; id++) {
    ax12SetRegister( id, AX_LED, 1); //for on
    Serial.print("Motor ID IS: ");
    Serial.println(id);
    delay(5000);  
    ax12SetRegister( id, AX_LED, 0); //for off
    delay(10);  
  }
}
*/

int LEDIDold, LEDID, loc1, loc2;
String readString, substring;
void loop() {

  //expect a string like "LED 1k"
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    //if (c == '\n') {  //looks for end of data packet marker
    if (c == 'k') {
      Serial.println(readString); //prints string to serial port out
      //do stuff      
      loc1 = readString.indexOf("LED");
      loc2 = readString.indexOf("k");
      substring = readString.substring(loc1+4, loc2);
      LEDID=substring.toInt();
      for (int id = 0; id <= 255; id++) {
        ax12SetRegister( id, AX_LED, 0);
      }
      delay(2000);
      /*
      ax12SetRegister( LEDIDold, AX_LED, 0); //for off
      Serial.print("Turning Off LED ");
      Serial.println(LEDIDold);
      */
      ax12SetRegister( LEDID, AX_LED, 1); //for on
      Serial.print("Turning ON LED ");
      Serial.println(LEDID);
      readString=""; //clears variable for new input
      substring=""; 
      }  
    else {     
      readString += c; //continue reading and adding until k is reached
    }
  }
}

//I'm only writing this because I'm sick of wading through years-old poorly documented code that's meant to do it for me.
//It might be a shit job but it'll at least work.


#include <ax12.h>
#include <BioloidController.h>
//#include "poses.h"

BioloidController bioloid = BioloidController(1000000);

const int SERVOCOUNT = 12;
int id;
int pos;
boolean IDCheck;
boolean RunCheck;


// TO TURN ON OR OFF THE LED. Use this to work out servo ID's...
//ax12SetRegister( id, AX_LED, 1) //for on
//ax12SetRegister( id, AX_LED, 0) //for off
//zoomkat 3-5-12 simple delimited ',' string  
//from serial port input (via serial monitor)
//and print result out serial port

String readString, substring;
int loc1; 
int loc2, Position1, Position2, Position3, Position4, Position5, Position6, Position7, Position8, Position9, Position10, Position11, Position12, Position13, Position14, Position15, Position16;
void setup() {
  Serial.begin(38400);
  Serial.println("serial delimit test 1.0"); // so I can keep track of what is loaded
}

void loop() {

  //expect a string like "#1 P100 #2 P200 .... #12 P500k"
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    //if (c == '\n') {  //looks for end of data packet marker
    if (c == 'k') {
      Serial.println(readString); //prints string to serial port out
      //do stuff      
      loc1 = readString.indexOf("#1 P");
      loc2 = readString.indexOf("#2");
      substring = readString.substring(loc1+4, loc2);
      Position1=substring.toInt();

      loc1 = readString.indexOf("#2 P");
      loc2 = readString.indexOf("#3");
      substring = readString.substring(loc1+4, loc2);
      Position2=substring.toInt();

      loc1 = readString.indexOf("#3 P");
      loc2 = readString.indexOf("#4");
      substring = readString.substring(loc1+4, loc2);
      Position3=substring.toInt();

      loc1 = readString.indexOf("#4 P");
      loc2 = readString.indexOf("#5");
      substring = readString.substring(loc1+4, loc2);
      Position4=substring.toInt();

      loc1 = readString.indexOf("#5 P");
      loc2 = readString.indexOf("#6");
      substring = readString.substring(loc1+4, loc2);
      Position5=substring.toInt();

      loc1 = readString.indexOf("#6 P");
      loc2 = readString.indexOf("#7");
      substring = readString.substring(loc1+4, loc2);
      Position6=substring.toInt();
      
      loc1 = readString.indexOf("#7 P");
      loc2 = readString.indexOf("#8");
      substring = readString.substring(loc1+4, loc2);
      Position7=substring.toInt();

      loc1 = readString.indexOf("#8 P");
      loc2 = readString.indexOf("#9");
      substring = readString.substring(loc1+4, loc2);
      Position8=substring.toInt();

      loc1 = readString.indexOf("#9 P");
      loc2 = readString.indexOf("#10");
      substring = readString.substring(loc1+4, loc2);
      Position9=substring.toInt();
      
      loc1 = readString.indexOf("#10 P");
      loc2 = readString.indexOf("#11");
      substring = readString.substring(loc1+5, loc2);
      Position10=substring.toInt();

      loc1 = readString.indexOf("#11 P");
      loc2 = readString.indexOf("#12");
      substring = readString.substring(loc1+5, loc2);
      Position11=substring.toInt();

      loc1 = readString.indexOf("#12 P");
      loc2 = readString.indexOf("k");
      substring = readString.substring(loc1+5, loc2);
      Position12=substring.toInt();
      //Position13=1023-Position2;
      //Position14=1023-Position5;
      //Position15=1023-Position15;
      //Position16=1023-Position16;

      readString=""; //clears variable for new input
      substring=""; 
      SetPosition(1, Position1);
      SetPosition(2, Position2);
      SetPosition(3, Position3);
      SetPosition(4, Position4);
      SetPosition(5, Position5);
      SetPosition(6, Position6);
      SetPosition(7, Position7);
      SetPosition(8, Position8);
      SetPosition(9, Position9);
      SetPosition(10, Position10);
      SetPosition(11, Position11);
      SetPosition(12, Position12);
      //SetPosition(13, Position13);
      //SetPosition(14, Position14);
      //SetPosition(15, Position15);
      //SetPosition(16, Position16);
    }  
    else {     
      readString += c; //continue reading and adding until k is reached
    }
  }
}

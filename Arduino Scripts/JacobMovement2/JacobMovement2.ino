// Example 3 - Receive with start- and end-markers
#include <ax12.h>
#include <BioloidController.h>
//#include "poses.h"
BioloidController bioloid = BioloidController(1000000);

String readString, substring;
int loc1; 
int loc2, Position1, Position2, Position3, Position4, Position5, Position6, Position7, Position8, Position9, Position10, Position11, Position12, Position13, Position14, Position15, Position16;



const byte numChars = 100;
char receivedChars[numChars];

boolean newData = false;

void setup() {
    Serial.begin(38400);
    Serial.println("<Arduino is ready>");
}

void loop() {
    recvWithStartEndMarkers();
    showNewData();
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        String str(receivedChars);
        loc1 = str.indexOf("#1 P");
        loc2 = str.indexOf("#2");
        substring = str.substring(loc1+4, loc2);
        Position1=substring.toInt();
        
        loc1 = str.indexOf("#2 P");
        loc2 = str.indexOf("#3");
        substring = str.substring(loc1+4, loc2);
        Position2=substring.toInt();
  
        loc1 = str.indexOf("#3 P");
        loc2 = str.indexOf("#4");
        substring = str.substring(loc1+4, loc2);
        Position3=substring.toInt();
  
        loc1 = str.indexOf("#4 P");
        loc2 = str.indexOf("#5");
        substring = str.substring(loc1+4, loc2);
        Position4=substring.toInt();
  
        loc1 = str.indexOf("#5 P");
        loc2 = str.indexOf("#6");
        substring = str.substring(loc1+4, loc2);
        Position5=substring.toInt();
  
        loc1 = str.indexOf("#6 P");
        loc2 = str.indexOf("#7");
        substring = str.substring(loc1+4, loc2);
        Position6=substring.toInt();
        
        loc1 = str.indexOf("#7 P");
        loc2 = str.indexOf("#8");
        substring = str.substring(loc1+4, loc2);
        Position7=substring.toInt();
  
        loc1 = str.indexOf("#8 P");
        loc2 = str.indexOf("#9");
        substring = str.substring(loc1+4, loc2);
        Position8=substring.toInt();
  
        loc1 = str.indexOf("#9 P");
        loc2 = str.indexOf("#10");
        substring = str.substring(loc1+4, loc2);
        Position9=substring.toInt();
        
        loc1 = str.indexOf("#10 P");
        loc2 = str.indexOf("#11");
        substring = str.substring(loc1+5, loc2);
        Position10=substring.toInt();
  
        loc1 = str.indexOf("#11 P");
        loc2 = str.indexOf("#12");
        substring = str.substring(loc1+5, loc2);
        Position11=substring.toInt();
  
        loc1 = str.indexOf("#12 P");
        loc2 = str.indexOf("k");
        substring = str.substring(loc1+5, loc2);
        Position12=substring.toInt();
        Position13=1023-Position2;
        Position14=1023-Position5;
        Position15=1023-Position15;
        Position16=1023-Position16;
        
        Serial.print("Motor 1 Position ");
        Serial.println(Position1);
        Serial.print("Motor 2 Position ");
        Serial.println(Position2);
        Serial.print("Motor 3 Position ");
        Serial.println(Position3);
        Serial.print("Motor 4 Position ");
        Serial.println(Position4);
        Serial.print("Motor 5 Position ");
        Serial.println(Position5);
        Serial.print("Motor 6 Position ");
        Serial.println(Position6);
        Serial.print("Motor 7 Position ");
        Serial.println(Position7);
        Serial.print("Motor 8 Position ");
        Serial.println(Position8);
        Serial.print("Motor 9 Position ");
        Serial.println(Position9);
        Serial.print("Motor 10 Position ");
        Serial.println(Position10);
        Serial.print("Motor 11 Position ");
        Serial.println(Position11);
        Serial.print("Motor 12 Position ");
        Serial.println(Position12);
        
        newData = false;
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
        /*
        SetPosition(13, Position13);
        SetPosition(14, Position14);
        SetPosition(15, Position15);
        SetPosition(16, Position16);
        */
        Serial.write("command received thank you");
    }
}




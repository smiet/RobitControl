// Test to move all motors to 500 position

#include <ax12.h>
#include <BioloidController.h>


void setup() {
  Serial.begin(38400);
  Serial.println("serial delimit test 1.0"); // so I can keep track of what is loaded
}

void loop() {
  for (int id = 0; id <= 100; id++) {
    ax12SetRegister( id, AX_LED, 1); //for on
    Serial.print("Motor ID IS: ");
    Serial.println(id);
    delay(200);  
    //ax12SetRegister( id, AX_LED, 0); //for off
    delay(10);  
  }
}


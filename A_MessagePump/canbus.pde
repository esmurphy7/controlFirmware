#include "canbus.h"

//Fake hardcoded ID for now 
// Needs to be determined programatically
char ARDUINO_ID = 0x66;
char sender_id = 0x00;
char args[6];


// getCommand()
// Check if there is a commnad available and return it's id
// Abstraction function for whatever canBus stuff
// Needs to go here, 
//Returns the command ID if one is available, 0 otherwise.
char getCommand() {
  if(Serial.available() >= 8) {
    // Read the full command from the bus
    sender_id = Serial.read(); // Arduino ID
    char cmd = Serial.read(); // Command
    args[0] = Serial.read(); // Argument 1 for command
    args[1] = Serial.read(); // Argument 2 for command
    args[2] = Serial.read(); // Argument 3 for command
    args[3] = Serial.read(); // Argument 4 for command
    args[4] = Serial.read(); // Argument 5 for command
    args[5] = Serial.read(); // Reserved for later use
    Serial.println(sender_id);
    Serial.println(cmd);
    if(sender_id == ARDUINO_ID) {
      return cmd;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

// Abstraction function for canbus
//
void writeResponse(char cmd,
                   char a1,
                   char a2,
                   char a3,
                   char a4,
                   char a5,
                   char res) {
  Serial.write(ARDUINO_ID);
  Serial.write(cmd);
  Serial.write(a1);
  Serial.write(a2);
  Serial.write(a3);
  Serial.write(a4);
  Serial.write(a5);
  Serial.write(res);
}

// Abstraction function for canbus
//
char getArg(int number) {
  return args[number];
}

// Return the Arduino ID if available
//
char getID() {
  return ARDUINO_ID;
}

// Return the sender of the last recieved command
//
char getSenderID() {
  return sender_id;
}

void generateID(char seed) {
  if(ARDUINO_ID == 0x00) {
    ARDUINO_ID = seed+1;
  }
}

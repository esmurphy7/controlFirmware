#include "command.h"

// Send STATE_ALIVE on all serial ports
void broadcast() {
  // The Arduino should only be asked to broadcast STATE_ALIVE
  // if the snake is in the initialization phase
  // Generate an ID before it is sent
  generateID(getSenderID());
  
  // Send on Serial1
  Serial1.write(getID());
  Serial1.write(STATE_ALIVE);
  Serial1.write('H');
  Serial1.write('E');
  Serial1.write('L');
  Serial1.write('L');
  Serial1.write('O');
  Serial1.write('!');
  
  // Send on Serial2
  Serial2.write(getID());
  Serial2.write(STATE_ALIVE);
  Serial2.write('H');
  Serial2.write('E');
  Serial2.write('L');
  Serial2.write('L');
  Serial2.write('O');
  Serial2.write('!');
  
  // Send on Serial3
  Serial3.write(getID());
  Serial3.write(STATE_ALIVE);
  Serial3.write('H');
  Serial3.write('E');
  Serial3.write('L');
  Serial3.write('L');
  Serial3.write('O');
  Serial3.write('!');
  
  // Send on canbus
  writeResponse(STATE_ALIVE,'H','E','L','L','O','!');
}

#include "interface.h"
#include "canbus.h"
#include "command.h"

Interface::Interface() {
  _id = 0x00;
  arg[0] = 0x00;
  arg[1] = 0x00;
  arg[2] = 0x00;
  arg[3] = 0x00;
  arg[4] = 0x00;
  arg[5] = 0x00;
}

void Interface::initializeSlave(Canbus canbus) {
  // For now hardcode the id
  this->canbus = canbus;
  _id = 0x66;
  return;
  
  while(_id == 0x00) {
    // Listen on Serial2 for an id
    if(Serial2.available() >= 2) {
      _id = Serial2.read() + 1;
      // Forward the command
      sendDown();
    }
  } 
}

char Interface::getID() {
  return _id;
}

// getCommand()
// Check if there is a commnad available and return it's id
// Returns the command ID if one is available, CMD_NONE otherwise
Command Interface::getCommand() {
  if(canbus.available() >= 8) {
    // Read the full command from the bus
    char dest_id = canbus.read(); // Arduino ID
    cmd = (Command)canbus.read(); // Command
    arg[0] = canbus.read(); // Argument 1 for command
    arg[1] = canbus.read(); // Argument 2 for command
    arg[2] = canbus.read(); // Argument 3 for command
    arg[3] = canbus.read(); // Argument 4 for command
    arg[4] = canbus.read(); // Argument 5 for command
    arg[5] = canbus.read(); // Reserved for later use
    if(dest_id == _id) { // Make sure this arduino is meant to read it
      return cmd;
    } else {
      return CMD_NONE;
    }
  } else {
    return CMD_NONE;
  }
}

char Interface::getArg(int id) {
  return arg[id];
}
void Interface::sendCommand(char id, Command cmd, char arg1, char arg2, char arg3, 
                                                  char arg4, char arg5, char res) {
  canbus.write(id);
  canbus.write(cmd);
  canbus.write(arg1);
  canbus.write(arg2);
  canbus.write(arg3);
  canbus.write(arg4);
  canbus.write(arg5);
  canbus.write(res);
}
void Interface::sendState(State state, char arg1, char arg2, char arg3, 
                                       char arg4, char arg5, char res) {
  canbus.write(_id);
  canbus.write(state);
  canbus.write(arg1);
  canbus.write(arg2);
  canbus.write(arg3);
  canbus.write(arg4);
  canbus.write(arg5);
  canbus.write(res);
}

void Interface::sendUp() {
  Serial2.write(_id);
  Serial2.write(STATE_ALIVE);
}

void Interface::sendDown() {
  Serial1.write(_id);
  Serial1.write(STATE_ALIVE);
}

void Interface::broadcast() {
  sendUp();
  sendDown();
  sendState(STATE_ALIVE, PAD, PAD, PAD, PAD, PAD, PAD);
}

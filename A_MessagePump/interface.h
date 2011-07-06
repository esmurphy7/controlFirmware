#pragma once
#include "MCP2515_defs.h"
#include "MCP2515.h"

enum Command {
  CMD_NONE      =  0x00,
  CMD_SET_HORIZ =  0x61,
  CMD_SET_VERT  =  0x62,
  CMD_SET_MOTOR =  0x63,
  CMD_STOP      =  0x64,
  CMD_START     =  0x65,
  CMD_STANDBY   =  0x66,
  CMD_SET_PID   =  0x67,
  CMD_SET_SENSR =  0x68,
  CMD_SEND_BAT  =  0x69,
  CMD_SEND_ID   =  0x6C,
  CMD_BROADCAST =  0x6D
};

enum State {
  STATE_HORIZ     = 0x61,
  STATE_VERT      = 0x62,
  STATE_MOTOR     = 0x63,
  STATE_PID       = 0x67,
  STATE_SENSR     = 0x68,
  STATE_BAT       = 0x69,
  STATE_SEND_ID   = 0x6C,
  STATE_ALIVE     = 0x6D
};

#define PAD 'q'

class Interface {
  public:
    Interface (int);
    void init();
    Frame getMessage();
    
    char getID();
    Command getCommand();
    char getArg(int id);
    void sendCommand(char id, 
                     Command cmd, 
                     char arg1, 
                     char arg2, 
                     char arg3, 
                     char arg4 ,
                     char arg5, 
                     char res);
    void sendState(State state,
                   char arg1, 
                   char arg2, 
                   char arg3, 
                   char arg4,
                   char arg5, 
                   char res);
    void sendUp();
    void sendDown();
    void broadcast();
  private:
    MCP2515 CAN;
    byte _id;      // This Arduino's ID
    Command cmd;   // Last Command
    char arg[6];   // Last Command's arguments
};

#pragma once
#include "MCP2515_defs.h"
#include "MCP2515.h"

enum Command {
  CMD_NONE      =  0x00,//
  CMD_SET_HORIZ =  0x61,//
  CMD_SET_VERT  =  0x62,//
  CMD_SET_MOTOR =  0x63,//
  CMD_SEND_BAT  =  0x64,//
  CMD_SET_PID   =  0x65,//
  CMD_SET_SENSR =  0x66,//
  CMD_BROADCAST =  0x67,//
  CMD_STOP      =  0x68,//
  CMD_START     =  0x69,//
  CMD_STANDBY   =  0x6A,//
  CMD_DIE       =  0x6C,//
  CMD_DATA_DUMP =  0x6D,//
  CMD_PID_CAL   =  0x6E,//
  CMD_SET_DIE	=  0x71,//
  CMD_PRINT_V	=  0x72,//
  CMD_PRINT_H	=  0x73,//
  CMD_PRINT_MOT =  0x74,//
  CMD_GET_PID   =  0x75,//
  CMD_SET_ANGLEX=  0x76,
  CMD_SET_ANGLEZ=  0x77,
  CMD_GET_ANGLEX=  0x78,
  CMD_GET_ANGLEZ=  0x79,
  CMD_REINIT	=  0x7A
};

enum State {
  STATE_NONE	  = 0x51, //this is special initial state that goes into reserved
  STATE_HORIZ     = 0x61,
  STATE_VERT      = 0x62,
  STATE_MOTOR     = 0x63,
  STATE_BAT       = 0x64,
  STATE_PID       = 0x65,
  STATE_SENSR     = 0x66,//limit switch tripped
  STATE_ALIVE     = 0x67,//the state of broadcast. 
//Behold, the 4 reserved running states
  STATE_STOPPED   = 0x68,
  STATE_GOING     = 0x69,
  STATE_STANDBY   = 0x6A,//initial pause
  STATE_PAUSED    = 0x6B,//snake has been on but inactive for some amount of time
//
  STATE_DEAD      = 0x6C,//last communication before total shutdown
  STATE_DUMPING   = 0x6D,
  STATE_PID_CAL	  = 0x6E,
  STATE_BAT_SET	  = 0x71,
  
  STATE_ANGLEX	  = 0x76,
  STATE_ANGLEZ	  = 0x77,
  
  STATE_AIN		  = 0x78,
  STATE_AOUT	  = 0x79,
  
  STATE_INITING	  =	0x7A
};

#define PAD 0xff
#define MASTER 1
#define SLAVE  2

class Interface {
  public:
    Interface (int);
    void init(int);
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

#pragma once
#include "canbus.h"
#include "command.h"

char PAD = 'q';

class Interface {
  public:
    Interface ();
    void initializeSlave(Canbus canbus);
    
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
    Canbus canbus; // Canbus interface
    char _id;      // This Arduino's ID
    Command cmd;   // Last Command
    char arg[6];   // Last Command's arguments
};

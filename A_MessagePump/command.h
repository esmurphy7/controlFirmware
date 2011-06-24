#pragma once

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
  STATE_SEND_ID   = 0x6C,
  STATE_ALIVE     = 0x6D
};

void broadcast();
  
void setHoriz(char,char,char,char,char);
void setVert(char,char,char,char,char);
void setMotor(char,char,char,char,char);
void stopSnake();
void startSnake();
void standbySnake();
void setPID(char,char,char);
void setSensor(char,char,char,char,char);
  
char getBattery();

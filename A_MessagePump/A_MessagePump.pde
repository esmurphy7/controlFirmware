// Command pump for slave Arduino
// Reads commands and responds as required
#include "command.h"
#include "canbus.h"

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
}

void loop()
{
  char newCommand = getCommand();
  switch(newCommand) {
    case CMD_SET_HORIZ:
      writeResponse(0x61,'q','q','q','q','q','q');
      break;
    case CMD_SET_VERT:
      break;
    case CMD_SET_MOTOR:
      break;
    case CMD_STOP:
      break;
    case CMD_START:
      break;
    case CMD_STANDBY:
      break;
    case CMD_SET_PID:
      break;
    case CMD_SET_SENSR:
      break;
    case CMD_SEND_BAT:
      break;
    case CMD_SEND_ID:
      writeResponse(STATE_SEND_ID,getID(),'q','q','q','q','q');
      break;
    case CMD_BROADCAST:
      broadcast();
      break;
  }
  delay(50);
}

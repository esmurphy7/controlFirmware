// Command pump for slave Arduino
// Reads commands and responds as required
#include "command.h"
#include "canbus.h"
#include "interface.h"

Canbus bus;
Interface interface;

void setup()
{
  bus = Canbus();
  bus.begin(115200);
  interface = Interface();
  interface.initializeSlave(bus);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
}

void loop()
{
  switch(interface.getCommand()) {
    case CMD_NONE:
      break;
    case CMD_SET_HORIZ:
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
      interface.sendState(STATE_SEND_ID,interface.getID(),PAD,PAD,PAD,PAD,PAD);
      break;
    case CMD_BROADCAST:
      interface.broadcast();
      break;
  }
  delay(50);
}

// Command pump for slave Arduino
// Reads commands and responds as required
#include "interface.h"
#include "MCP2515.h"
#include <SPI.h>

#define CS_PIN 53

Interface interface(CS_PIN);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  interface.init();
  delay(10);
}

Frame command;

void loop()
{
  command = interface.getMessage();
  switch(command.data[0]) {
    case CMD_NONE:
      //Serial.println("CMD_NONE");
      break;
    case CMD_SET_HORIZ:
      //Serial.println("CMD_SET_HORIZ");
      break;
    case CMD_SET_VERT:
      //Serial.println("CMD_SET_VERT");
      break;
    case CMD_SET_MOTOR:
      //Serial.println("CMD_SET_MOTOR");
      break;
    case CMD_STOP:
      //Serial.println("CMD_STOP");
      break;
    case CMD_START:
      //Serial.println("CMD_START");
      break;
    case CMD_STANDBY:
      //Serial.println("CMD_STANDBY");
      break;
    case CMD_SET_PID:
      //Serial.println("CMD_SET_PID");
      break;
    case CMD_SET_SENSR:
      //Serial.println("CMD_SET_SENSR");
      break;
    case CMD_SEND_BAT:
      //Serial.println("CMD_SEND_BAT");
      break;
    case CMD_SEND_ID:
      //Serial.println("CMD_SEND_ID");
      //interface.sendState(STATE_SEND_ID,interface.getID(),PAD,PAD,PAD,PAD,PAD);
      break;
    case CMD_BROADCAST:
      //Serial.println("CMD_BROADCAST");
      //interface.broadcast();
      break;
  }
  //delay(50);
}

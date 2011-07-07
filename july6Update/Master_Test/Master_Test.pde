// Command pump for slave Arduino
// Reads commands and responds as required
#include "interface.h"
#include "MCP2515.h"
#include <SPI.h>

#define CS_PIN 53
#define NUM_SLAVES 3

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
  
  // Listen for slave broadcasts
  Frame slave_message;
  while(1) {
    slave_message = interface.getMessage();
    if(slave_message.id == 0x00) {
      continue;
    }
    Serial.print("Recieved Slave broadcast from id ");
    Serial.println(slave_message.id,HEX);
    if(slave_message.id == NUM_SLAVES) {
      break;
    }
  }
  delay(100);
  interface.sendCommand(0x02,CMD_SET_PID,0x60,0x02,0x01,PAD,PAD,PAD);
  Serial.println("Finished Setup");
}

Frame slaveSpeak;

void loop()
{
  interface.sendCommand(0x02,CMD_SET_HORIZ,0x20,0x50,0x80,0xA0,0xC0,PAD);
  delay(100);
  slaveSpeak.getMessage();
  switch(slaveSpeak.data[0]) {
    Serial.print(slaveSpeak.id,HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[0],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[1],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[2],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[3],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[4],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[5],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[6],HEX);
    Serial.print(" ");
    Serial.print(slaveSpeak.data[7],HEX);
    Serial.println();
  }
  delay(3000);
}

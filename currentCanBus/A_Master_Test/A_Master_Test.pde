// Command pump for slave Arduino
// Reads commands and responds as required
#include <interface.h>
#include <MCP2515.h>
#include <SPI.h>

#define CS_PIN 53

Interface interface(CS_PIN);
byte init_enable = 0x00;
byte num_slaves = 0x00;

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
  
  // Wait for initialization command from controller
  while (Serial3.available() < 2) {
    delay(10);
  }
  
  init_enable = Serial3.read();
  num_slaves = Serial3.read();
  
  interface.init(MASTER);
  delay(10);  
  // Listen for slave broadcasts
  Frame slave_message;
  while(init_enable) {
    slave_message = interface.getMessage();
    if(slave_message.id == 0x00) {
      continue;
    }
    Serial.print("Recieved Slave broadcast from id ");
    Serial.println(slave_message.id,HEX);
    if(slave_message.id == num_slaves) {
      break;
    }
  }
  delay(100);
  Serial.println("Finished Setup");
}

Frame slaveSpeak;
byte seg[5];

void loop()
{
  // Attempt at a sin wave
  /*
  seg[0] = (120*sin( (millis())/1000.0 ) + 120);
  seg[1] = (120*sin( (millis()+100)/1000.0 ) + 120);
  seg[2] = (120*sin( (millis()+200)/1000.0 ) + 120);
  seg[3] = (120*sin( (millis()+300)/1000.0 ) + 120);
  seg[4] = (120*sin( (millis()+400)/1000.0 ) + 120);
  Serial.print(seg[0],DEC);
  Serial.print(" ");
  Serial.print(seg[1],DEC);
  Serial.print(" ");
  Serial.print(seg[2],DEC);
  Serial.print(" ");
  Serial.print(seg[3],DEC);
  Serial.print(" ");
  Serial.print(seg[4],DEC);
  Serial.print(" ");
  interface.sendCommand(0x02,CMD_SET_HORIZ,seg[0],seg[1],seg[2],seg[3],seg[4],PAD);
  delay(100);*/
  
  // Pump slave messages
  slaveSpeak = interface.getMessage();
  if(slaveSpeak.id) {
    // Special Master handling for each command
   // switch(slaveSpeak.data[0]) {
    // Print debug
    /*  default:
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
        Serial.println();*/
  //  }
    // Forward message to controller
    Serial3.print(slaveSpeak.id);
    Serial3.print(slaveSpeak.data[0]);
    Serial3.print(slaveSpeak.data[1]);
    Serial3.print(slaveSpeak.data[2]);
    Serial3.print(slaveSpeak.data[3]);
    Serial3.print(slaveSpeak.data[4]);
    Serial3.print(slaveSpeak.data[5]);
    Serial3.print(slaveSpeak.data[6]);
    Serial3.print(slaveSpeak.data[7]);
  }
  
  // Pump XBee messages
  // Recieved 5 bytes per slave plus one command byte
  if(Serial3.available() >= 8) {
    byte slave_number = Serial.read();
    byte slave_command = Serial.read();
    Serial3.println("=====");
    Serial3.println(slave_command,HEX);
    Serial3.println("=====");
  //  for(byte i = 1; i <= num_slaves; i++) {
      // Forward the command with 5 single byte args
      interface.sendCommand(slave_number,(Command)slave_command,Serial3.read(),
                                                     Serial3.read(),
                                                     Serial3.read(),
                                                     Serial3.read(),
                                                     Serial3.read(),
                                                     Serial3.read());//should be pad
      delay(5);
   // }
  }
}

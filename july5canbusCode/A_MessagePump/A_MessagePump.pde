// Command pump for slave Arduino
// Reads commands and responds as required
#include "interface.h"
#include "MCP2515.h"
#include <SPI.h>
#include "PIDcontrol.h" //Julian PID header

#define CS_PIN 53

Interface interface(CS_PIN);

//PID integration
/*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//pin numbers CHECK FOR ACTUAL PINS
const int sensorPinX[] = {1, 2, 3, 4, 5};
const int sensorPinZ[] = {6, 7, 8, 9, 10};
const int actuatorPinXL[] = {2, 3, 4, 7, 8};
const int actuatorPinXR[] = {9, 10, 11, 12, 13};
/*
const int actuatorPinZU[] = {21, 22, 23, 24, 25};
const int actuatorPinZD[] = {26, 27, 28, 29, 30};
*/
//other pins here
//const int pumpPin

//PID controllers
PIDcontrol PIDcontrollerX[5] = {
  PIDcontrol( sensorPinX[0], actuatorPinXL[0], actuatorPinXR[0]),
  PIDcontrol( sensorPinX[1], actuatorPinXL[1], actuatorPinXR[1]),
  PIDcontrol( sensorPinX[2], actuatorPinXL[2], actuatorPinXR[2]),
  PIDcontrol( sensorPinX[3], actuatorPinXL[3], actuatorPinXR[3]),
  PIDcontrol( sensorPinX[4], actuatorPinXL[4], actuatorPinXR[4])
};
/*
PIDcontrol PIDcontrollerZ[5]= {
  PIDcontrol( sensorPinZ[0], actuatorPinZU[0], actuatorPinZD[0]),
  PIDcontrol( sensorPinZ[1], actuatorPinZU[1], actuatorPinZD[1]),
  PIDcontrol( sensorPinZ[2], actuatorPinZU[2], actuatorPinZD[2]),
  PIDcontrol( sensorPinZ[3], actuatorPinZU[3], actuatorPinZD[3]),
  PIDcontrol( sensorPinZ[4], actuatorPinZU[4], actuatorPinZD[4])
};
*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
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
  
  //Julian PID integration
  /*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
    //turn off actuators and set pin modes
  for(int i=0;i<5;i++){
    pinMode(actuatorPinXL[i], OUTPUT);
    analogWrite(actuatorPinXL[i], 0);
    pinMode(actuatorPinXR[i], OUTPUT);
    analogWrite(actuatorPinXR[i], 0);
    /*
    pinMode(actuatorPinZU[i], OUTPUT);
    analogWrite(actuatorPinZU[i], 0);
    pinMode(actuatorPinZD[i], OUTPUT);
    analogWrite(actuatorPinZD[i], 0);
    */
  }
  delay(100);
  //make set point current value(so it dosen't move)
  for(int i=0;i<5;i++){
    PIDcontrollerX[i].setSetPoint(analogRead(sensorPinX[i]));
    /*
    PIDcontrollerz[i].setSetPoint(analogRead(sensorPinZ[i]));
    */
  }
  /*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
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
      for (int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setSetPoint(0|command.data[i]);
        PIDcontrollerX[i-1].debugging();
            Serial.println();
      }
      interface.sendState(STATE_HORIZ,PIDcontrolX[0],PIDcontrolX[1],PIDcontrolX[2],PIDcontrolX[3],PIDcontrolX[4]);
      break;
    case CMD_SET_VERT:
      //Serial.println("CMD_SET_VERT");
      break;
    case CMD_SET_MOTOR:
      Serial.println("CMD_SET_MOTOR");
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
      Serial.println("CMD_SET_PID");
      for(int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setConstants(0|command.data[1],0|command.data[2],0|command.data[3]);
        Serial.println(i,HEX);
      }
      break;
    case CMD_SET_SENSR:
      Serial.println("CMD_SET_SENSR");
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
/*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  //use PID to make actuator adjustments
  //always run every cycle
  for(int i=0;i<5;i++){
    PIDcontrollerX[i].updateOutput();
    /*
    PIDcontrollerZ[i].updateOutput();
    */
  }
/*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  //delay(50);
}

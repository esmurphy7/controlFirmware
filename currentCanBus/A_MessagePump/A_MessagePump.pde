// Command pump for slave Arduino
// Reads commands and responds as required
#include "interface.h"
#include "MCP2515.h"
#include <SPI.h>
#include "PIDcontrol.h" //Julian PID header

#define CS_PIN 53

Interface interface(CS_PIN);

/*X is for horizontal
  Z is for vertical*/
 
#define  DITHERPWM  127

//THESE PINS WERE CHECKED ON JULY 20 2011
//analog pins for pot reading
const int sensorPinX[] = { 0, 1, 2, 3, 4 };
const int sensorPinZ[] = { 5, 6, 7, 8, 9 };

//PID signal pins
const int actuatorPinX[] = { 3, 4, 5, 6, 7 };
const int actuatorPinZ[] = { 8, 9, 10, 11, 12 };

//Valve select pins because we don't have enough PWM
const int valveSelectX[] = { 36, 37, 38, 39, 40 };
const int valveSelectZ[] = { 41, 42, 43, 44, 45 };

//Dither will be fed into all unselected valves
const int ditherPin = 13;

//motor write speed
const int motorPin = 2;

motorController	motorControl(motorPin);

/*limit switches in series. will determine L/R through math*/
const int limit_switchX[] = { 26, 27, 28, 29, 30 };
const int limit_switchZ[] = { 31, 32, 33, 34, 35 };

//have to check battery voltage
const int batPin = 00;

//arduinos need to kill

//PID controllers
PIDcontrol PIDcontrollerX[5] = {
  PIDcontrol( sensorPinX[0], actuatorPinX[0], limit_switchX[0], valveSelectX[0]),
  PIDcontrol( sensorPinX[1], actuatorPinX[1], limit_switchX[1], valveSelectX[1]),
  PIDcontrol( sensorPinX[2], actuatorPinX[2], limit_switchX[2], valveSelectX[2]),
  PIDcontrol( sensorPinX[3], actuatorPinX[3], limit_switchX[3], valveSelectX[3]),
  PIDcontrol( sensorPinX[4], actuatorPinX[4], limit_switchX[4], valveSelectX[4])
};
/*
PIDcontrol PIDcontrollerZ[5] = {
  PIDcontrol( sensorPinZ[0], actuatorPinZ[0], limit_switchZ[0], valveSelectZ[0]),
  PIDcontrol( sensorPinZ[1], actuatorPinZ[1], limit_switchZ[1], valveSelectZ[1]),
  PIDcontrol( sensorPinZ[2], actuatorPinZ[2], limit_switchZ[2], valveSelectZ[2]),
  PIDcontrol( sensorPinZ[3], actuatorPinZ[3], limit_switchZ[3], valveSelectZ[3]),
  PIDcontrol( sensorPinZ[4], actuatorPinZ[4], limit_switchZ[4], valveSelectZ[4])
};
*/

//defaul values for stuff
//motor
int motorValue = 127;

//PID calibrate
boolean pidCalibrated = false;

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
  
  int type = SLAVE;
  
  interface.init(type);
  delay(10);
  
  pinMode(motorPin, OUTPUT);
  
  //make sure actuators are off and set pin modes
  
  pinMode(ditherPin, OUTPUT);
  digitalWrite(ditherPin,DITHERPWM);

  for(int i=0;i<5;i++){
    pinMode(actuatorPinX[i], OUTPUT);
    analogWrite(actuatorPinX[i], 0);
    
    pinMode(valveSelectX[i], OUTPUT);
 
  /*  
    pinMode(actuatorPinZ[i], OUTPUT);
    analogWrite(actuatorPinZ[i], 0);
    
    pinMode(valveSelectZ[i], OUTPUT);
    */
  }
  
  delay(100);  
  
  //make set point current value(so it dosen't move)
  for(int i=0;i<5;i++){
    PIDcontrollerX[i].setSetPoint(analogRead(sensorPinX[i]));
    
    //PIDcontrollerZ[i].setSetPoint(analogRead(sensorPinZ[i]));
    
  }
  /*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
}

Frame command;

void loop(){
  command = interface.getMessage();
  switch(command.data[0]) {
    case CMD_NONE:
      //Serial.println("CMD_NONE");
      break;
    case CMD_SET_HORIZ:
        //Serial.println("CMD_SET_HORIZ");
    for (int i=1;i<=5;i++){
      PIDcontrollerX[i-1].setSetPoint(0|command.data[i]);
    }
    interface.sendState(STATE_HORIZ,PIDcontrollerX[0].getSensor(),PIDcontrollerX[1].getSensor(),PIDcontrollerX[2].getSensor(),PIDcontrollerX[3].getSensor(),PIDcontrollerX[4].getSensor(),PAD);
/*      Serial.print(PIDcontrollerX[0].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[1].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[2].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[3].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[4].getSensor(),DEC);
      Serial.println();*/
      break;
    case CMD_SET_VERT:
      //Serial.println("CMD_SET_VERT");
/*        for (int i=1;i<=5;i++){
          PIDcontrollerZ[i-1].setSetPoint(0|command.data[i]);
        }
      interface.sendState(STATE_VERT,PIDcontrollerZ[0].getSensor(),PIDcontrollerZ[1].getSensor(),PIDcontrollerZ[2].getSensor(),PIDcontrollerZ[3].getSensor(),PIDcontrollerZ[4].getSensor(),PAD);
      */break;
    case CMD_PRINT_H:
          interface.sendState(STATE_HORIZ,PIDcontrollerX[0].getSensor(),PIDcontrollerX[1].getSensor(),PIDcontrollerX[2].getSensor(),PIDcontrollerX[3].getSensor(),PIDcontrollerX[4].getSensor(),PAD);
    break;
    case CMD_PRINT_V:
  //          interface.sendState(STATE_VERT,PIDcontrollerZ[0].getSensor(),PIDcontrollerZ[1].getSensor(),PIDcontrollerZ[2].getSensor(),PIDcontrollerZ[3].getSensor(),PIDcontrollerZ[4].getSensor(),PAD);
    break;
    case CMD_SET_MOTOR:
      Serial.println("CMD_SET_MOTOR");
	  motorController.maxSet(command.data[1]);
      interface.sendState(STATE_MOTOR,command.data[1],motorController.getSpeed(),PAD,PAD,PAD,PAD);
      break;
    case CMD_PRINT_MOT:
      interface.sendState(STATE,MOTOR,motorController.getMax(),motorController.getSpeed(),PAD,PAD,PAD,PAD);
    break;
    case CMD_SEND_BAT:
      //Serial.println("CMD_SEND_BAT");
      //interface.sendState(STATE_BAT,___Read(batPin)
      break;
    case CMD_SET_DIE:// battery kill level
    
      break;
    case CMD_SET_PID:
      Serial.println("CMD_SET_PID");
      for(int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setConstants(0|command.data[1],0|command.data[2],0|command.data[3]);
 //     PIDcontrollerZ[i-1].setConstants(0|command.data[1],0|command.data[2],0|command.data[3]);
      }
      break;
   case CMD_GET_PID:
   
   break;
   case CMD_PID_CAL:
     
     pidCalibrated = true;
   break;
    case CMD_SET_SENSR://CRAP about limit switches. do we really need now?
      //Serial.println("CMD_SET_SENSR");
      break;
    case CMD_BROADCAST:
      //Serial.println("CMD_BROADCAST");
      //interface.broadcast();
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
   case CMD_DIE:
     //ack die and power down
      break;
   case CMD_DATA_DUMP://master command. not slave. really need here?
     //send back what needs to be sent
     break;

  }
/*++++++++++++++++++++++++++++++++++++++++++++++++++++*/

  if(pidCalibrated == true){
    //use PID to make actuator adjustments
    //always run every cycle
    for(int i=0;i<5;i++){
      PIDcontrollerX[i].updateOutput();
      /*
      PIDcontrollerZ[i].updateOutput();
      */
    }
  }
/*++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  //delay(50);
}

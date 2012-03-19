
#include "PIDcontrol.h"

//pin numbers
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

void setup(){
  Serial.begin(115200);
  
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
}


void loop(){
  //read messages
  
  //update set points
  
  
  //use PID to make actuator adjustments
  for(int i;i<5;i++){
    PIDcontrollerX[i].updateOutput();
    /*
    PIDcontrollerZ[i].updateOutput();
    */
  }
  
}





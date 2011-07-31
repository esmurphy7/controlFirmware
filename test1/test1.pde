#include "PIDcontrol.h" //Julian PID header
#include "motorControl.h"

/*X is for horizontal
  Z is for vertical*/
 
#define  DITHERPWM  127

//analog pins for pot reading
const int sensorPinX[] = {0, 1, 2, 3, 4}; //{ 0, 1, 2, 3, 4 };
//const int sensorPinZ[] = { 5, 6, 7, 8, 9 };

//PID signal pins
const int actuatorPinX[] = { 3, 5, 7, 9, 11 };
//const int actuatorPinZ[] = { 8, 9, 10, 11, 12 };

//Valve select pins because we don't have enough PWM
const int valveSelectX[] = { 36, 37, 38, 39, 40 };
//const int valveSelectZ[] = { 41, 42, 43, 44, 45 };

//Dither will be fed into all unselected valves
//const int ditherPin = 13;

//motor write speed
const int motorPin = 2;

motorControl	motorController(motorPin);

/*limit switches in series. will determine L/R through math*/
const int limit_switchX[] = { 26, 27, 28, 29, 30 };
//const int limit_switchZ[] = { 31, 32, 33, 34, 35 };

//have to check battery voltage
//const int batPin = 00;

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

void setup() {
  Serial.begin(115200);
  
  for(int i=0;i<5;i++){
    pinMode(limit_switchX[i], INPUT);
  }
  pinMode(motorPin,OUTPUT);
  digitalWrite(motorPin,0);
  
  boolean gogo = false;
  while(gogo==false){
    if(Serial.read() == 'p'){
      gogo = true;
    }
  }
  PIDcontrollerX[0].setConstants(3, 1, 2);
  delay(1000);
  Serial.println("main program");
}

byte cur_pid[5] = {127,127,127,127,127};

int outputX = 0;

void loop() {  
  if(Serial.available()){
    byte tmp = Serial.read();
    Serial.println(tmp);
    if(tmp=='a') {cur_pid[0]+=10;}
    if(tmp=='z') {cur_pid[0]-=10;}
    if(tmp=='s') {cur_pid[1]+=10;}
    if(tmp=='x') {cur_pid[1]-=10;}
    if(tmp=='d') {cur_pid[2]+=10;}
    if(tmp=='c') {cur_pid[2]-=10;}
    if(tmp=='f') {cur_pid[3]+=10;}
    if(tmp=='v') {cur_pid[3]-=10;}
    if(tmp=='g') {cur_pid[4]+=10;}
    if(tmp=='b') {cur_pid[4]-=10;}

    if(tmp=='q') {
      Serial.println(motorController.getSpeed()); 
    }
    if(tmp=='w'){
      Serial.println(motorController.getMax());
    }
    if(tmp=='e'){
      motorController.off();
      for(int i=0;i<5;i++){
        PIDcontrollerX[i].setSetPoint(analogRead(sensorPinX[i]));
      }
      Serial.println(motorController.getSpeed());
    }
    if(tmp=='r'){
      for(int i=0;i<5;i++){
        Serial.print(PIDcontrollerX[i].getSensor());
        Serial.print(" ");
      }
      Serial.println(" ");
    }
   /* if(tmp=='t'){
      if(Serial.read()<3){
        Serial.println("need 3 arguments");
      }
      else{
        for(ia
      }
    }*/
    if(tmp=='y'){
      Serial.print("200 speed");
      digitalWrite(motorPin,200);
      delay(1000);
      Serial.print("100 speed");
      digitalWrite(motorPin,100);
      delay(1000);
      Serial.print("0 speed");
      digitalWrite(motorPin,0);
      delay(1000);
      Serial.println("motor test done");
    }
    
    if(tmp=='u'){
      for(int i=0; i<5; i++){
        Serial.print(PIDcontrollerX[i].getSetpoint());
        Serial.print(" ");
      }
      Serial.println();
    }
    
    if(tmp == 'i'){
      char tmp2[3];
      for(int i=0;i<3;i++){
        tmp2[i]=Serial.read();
      }
      int tmp3=atoi(tmp2);
      Serial.print("atoi output");
      Serial.print(" ");
      Serial.print(tmp3);
      Serial.println(" ");
      analogWrite(motorPin,tmp3);
    }
    if(tmp=='o') {
      Serial.println(outputX);
    }
    if(tmp=='Q'){
      analogWrite(3,200);
      analogWrite(4,0);
      analogWrite(motorPin,150);
      delay(1000);
      analogWrite(4,200);
      analogWrite(3,0);
      delay(1000);
      analogWrite(motorPin,0);
    }
  }//end of tmp read

    for (int i=0;i<5;i++){
      PIDcontrollerX[i].setSetPoint(cur_pid[i]);
    }
   // for(int i=0;i<5;i++){
      outputX=PIDcontrollerX[0].updateOutput();
      motorController.updateAX(0,abs(outputX));
   // }
  motorController.updateMotor();
}

/*
August 6, 2011
test pid on snake
one segmet
*/
#include "PIDcontrol.h"
#include "boashield_pins.h"

boolean Stop = true;

//pin numbers
const int sensorPin = 11;//15
const int actuatorSpeedPin = 13;
const int actuatorDirectionPin = 31;
const int ditherPin = 6;
const int motorPin = 5;

const int motorSpeed = 120;

int setPoint = 100;

int lowRange = 0;
int highRange = 1023;

//PID controllers
PIDcontrol PIDcontroller 
    = PIDcontrol(sensorPin, actuatorSpeedPin, actuatorDirectionPin);

void setup(){
  Serial.begin(115200);
  
  //turn off actuators and set pin modes
  pinMode(actuatorSpeedPin, OUTPUT);
  analogWrite(actuatorSpeedPin, 0);
  pinMode(actuatorDirectionPin, OUTPUT);
  digitalWrite(actuatorDirectionPin, LOW);
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);
  
  delay(100);
  //make set point current value(so it dosen't move)
  setPoint = sensorPin;
  PIDcontroller.setSetPoint(analogRead(sensorPin));
}


void loop(){
  
  if(Serial.available()>0){
    
    switch(Serial.read()){
      case 's':
        //setpionts to come
        while(Serial.available()<1){
          delay(100);
        }
        setPoint = map(Serial.read(),0,255,lowRange,highRange);
        PIDcontroller.setSetPoint(setPoint);
        break;
      case 'c':
        //calabration
        calibration();
        Stop = true;
        Serial.flush();
        break;
      case 'k':
        //3 constants to be set
        while(Serial.available() < 3){
          delay(100);
        }
        PIDcontroller.setConstants(Serial.read(),Serial.read(),Serial.read());
        break;
      case '1':
        //start:
        Stop = false;
        //analogWrite(motorPin, motorSpeed);
        break;
      case '0':
        //stop
        Stop = true;
        Serial.flush();
        analogWrite(motorPin, 0);
        break;
    }
  }
  
  if(Stop == false){
    //use PID to make actuator adjustments
    PIDcontroller.updateOutput();
    
    if(abs(analogRead(sensorPin)-setPoint)<5){
      analogWrite(motorPin, 0);
    }
    else{
      analogWrite(motorPin, motorSpeed);
    }
  }
  else{
    analogWrite(actuatorSpeedPin, 0);
    digitalWrite(actuatorDirectionPin, LOW);
    analogWrite(motorPin, 0);
  }
  
  //return position
  Serial.write(constrain(map(analogRead(sensorPin),lowRange,highRange,0,255),0,255));
  //Serial.write(map(analogRead(BAT_LEVEL_24V),0,1023,0,255));
}

void calibration(){
  //calibrade low and high range on snake
  
  //move to high range
  analogWrite(motorPin,motorSpeed);
  analogWrite(actuatorSpeedPin,255);
  digitalWrite(actuatorDirectionPin,HIGH);
  delay(1500);
  //stop
  analogWrite(motorPin,0);
  analogWrite(actuatorSpeedPin,0);
  delay(1000);
  //setHighRange
  highRange = analogRead(sensorPin);
  
  //move to low range
  analogWrite(motorPin,motorSpeed);
  analogWrite(actuatorSpeedPin,255);
  digitalWrite(actuatorDirectionPin,LOW);
  delay(1500);
  //stop
  analogWrite(motorPin,0);
  analogWrite(actuatorSpeedPin,0);
  delay(1000);
  //setLowRange
  lowRange = analogRead(sensorPin);
  
  //turn off everything
  analogWrite(motorPin,0);
  analogWrite(actuatorSpeedPin,0);
  digitalWrite(actuatorDirectionPin,LOW);
}




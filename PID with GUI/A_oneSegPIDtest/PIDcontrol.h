/*
PID control library
calculates the PID output
and writes to output pin
*/
#ifndef PIDcontrol_h
#define PIDcontrol_h

#include "WProgram.h"

//class outputs PID control
//output constraint to between -255 and 255
class PIDcontrol{
private:
  //variables
  int sensorPin;
  int actuatorSpeedPin;
  int actuatorDirectionPin;
  
  int kp;
  int kd;
  int ki;
  
  int setPoint;
  
  int sensorReading;
  int error;
  int derivative;
  int integral;
  int output;
  unsigned long prevTime;
  int prevSensorReading;
public:
  //constructor
  PIDcontrol(int newSensorPin,int newActuatorSpeedPin,int newActuatorDirectionPin){
    sensorPin = newSensorPin;
    actuatorSpeedPin = newActuatorSpeedPin;
    actuatorDirectionPin = newActuatorDirectionPin;
    
    kp = 0;
    kd = 0;
    ki = 0;
    
    reset();
  }
  
  void reset(){
    setPoint = analogRead(sensorPin);
    derivative = -1;
    integral = 0;
    output = 0;
    prevTime = millis();
    prevSensorReading = -1;
  }
  
  void setConstants(int newKp, int newKd, int newKi){
    kp = newKp;
    kd = newKd;
    ki = newKi;
  }
  
  void setSetPoint(int newSetPoint){
    setPoint = newSetPoint;
  }
  
  int updateOutput(){
    sensorReading = analogRead(sensorPin);
    
    error = sensorReading - setPoint;
    
    //prevSensorReading initialized to -1
    //-1 if no sensor readins previously
    if(prevSensorReading != -1){
      derivative = (sensorReading - prevSensorReading) / (millis() - prevTime);
    }
    else{
      derivative = 0;
    }
    
    if(prevSensorReading != sensorReading){
      prevSensorReading = sensorReading;
      prevTime = millis();
    }
    
    //integral constraint and decrementing for anti-windup
    integral += error;
    if(integral > 0){
      integral -= 1;
    }
    else if(integral < 0){
      integral += 1;
    }
    integral = constrain(integral, -1024, 1024);
    
    //calculate output
    //adjust fomula to change sensativity to constaints
    output = kp*error - kd*derivative + ki*integral/128;
    output = constrain(output, -255, 255);
    
    
    //add dithering
    output = output + 20*int(sin(float(millis()/2)));
    output = constrain(output, -255, 255);
    
    if(output >= 0){
      analogWrite(actuatorSpeedPin, output);
      digitalWrite(actuatorDirectionPin, LOW);
    }
    else{
      analogWrite(actuatorSpeedPin, -output);
      digitalWrite(actuatorDirectionPin, HIGH);
    }
    
    return output;
  }
  
};

#endif

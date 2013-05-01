/*

  OneWireNXP.h - Communication with NXP One Wire devices 
  
  Created: April 30, 2013 
  Part of the titanaboa.ca project
  
  Decription: This code was built to communicate with an NXP
  KMA210 programmable angle sensor but it might work with
  other devices from NXP with a One-Wire Interface. Please note 
  that the One-Wire Interface spec from NXP is incompatible with 
  the popular 1-Wire spec from Maxim.

*/

#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#include <Arduino.h>

//class outputs PID control
//output constraint to between -255 and 255
class PIDcontrol{
private:
  //variables
  int sensorPin;
  int actuatorSpeedPin;
  int actuatorDirectionPin;
  boolean even;
  
  int kp;
  int kd;
  int ki;
  
  int setPoint;
  
  int sensorReading;
  int error;
  int derivative;
  int integral;
  int output;
  unsigned long prevSensorReadingTime;
  int prevSensorReading;
  unsigned long prevUpdateTime;
  
public:
  //constructor
  PIDcontrol(int newSensorPin,int newActuatorDirectionPin,int newActuatorSpeedPin){
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
    prevSensorReadingTime = millis();
    prevSensorReading = -1;
    prevUpdateTime = 0;
    analogWrite(actuatorSpeedPin, 0);
  }
  
  void setEven(boolean newEven){
    even = newEven;
  }
  
  boolean getEven(){
    return even;
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
    //sensor reading
    sensorReading = analogRead(sensorPin);
    
    prevUpdateTime = millis();
    
    
    //calculate error
    error = setPoint - sensorReading;
    
    //calculate current derivative
    //prevSensorReading initialized to -1
    //-1 if no sensor readins previously
    //Derivative calculated as points per 0.1 second(ms too small)
    if((millis()-prevSensorReadingTime) > 10){
      derivative = (sensorReading-prevSensorReading)*100/(int)(millis()-prevSensorReadingTime);
      if(abs(sensorReading-prevSensorReading)<=1){
        derivative = 0;
      }
      prevSensorReading = sensorReading;
      prevSensorReadingTime = millis();
    }
    
    //calculate integral
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
    output = constrain(output, -250, 250);
    
    
    //add dithering
    output = output + 20*int(sin(float(millis()/2)));
    output = constrain(output, -255, 255);
    
    //apply output
    if(output >= 0){
      analogWrite(actuatorSpeedPin, output);
      if(even){
        digitalWrite(actuatorDirectionPin, HIGH);
      }
      else{
        digitalWrite(actuatorDirectionPin, LOW);
      }
    }
    else{
      analogWrite(actuatorSpeedPin, -output);
      if(even){
        digitalWrite(actuatorDirectionPin, LOW);
      }
      else{
        digitalWrite(actuatorDirectionPin, HIGH);
      }
    }
    
    return output;
  }
  
};

#endif _PID_CONTROL_H_

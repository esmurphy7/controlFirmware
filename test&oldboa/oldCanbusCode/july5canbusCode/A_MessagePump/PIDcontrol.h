/*
PID control library
calculates the PID output
and writes to output pin
*/
#ifndef PIDcontrol_h
#define PIDcontrol_h

#include "WProgram.h"

//class outputs PID control
//out put constraint to between -255 and 255
class PIDcontrol{
private:
  //variables
  int sensorPin;
  int actuatorPinFwd;
  int actuatorPinRev;
  
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

  void debugging(){
    Serial.print(sensorReading,HEX);
    Serial.print(" ");
    Serial.print(setPoint,HEX);
    Serial.print(" ");
    Serial.print(output,HEX);
    Serial.print(" ");
    Serial.print(kp,HEX);
    Serial.print(" ");
    Serial.print(kd,HEX);
    Serial.print(" ");
    Serial.print(ki,HEX);
  }

  int getSensor(){
    return sensorReading;
  }


  //constructor
  PIDcontrol(int newSensorPin,int newActuatorPinFwd,int newActuatorPinRev){
    sensorPin = newSensorPin;
    actuatorPinFwd = newActuatorPinFwd;
    actuatorPinRev = newActuatorPinRev;
    
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
    //map sensor readings to 0-255 range
    sensorReading = map(sensorReading, 60, 120, 0, 255);
    
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
    
    //integral constraint for anti-windup
    integral += error;
    integral = constrain(integral, -1024, 1024);
    
    //calculate output
    //adjust fomula to change sensativity to constaints
    output = kp*error - kd*derivative*1024 + ki*integral/256;
    output = constrain(output, -235, 235);
    
    //add dithering
    output = output + 20*int(sin(float(millis()/2)));
    output = constrain(output, -255, 255);
    
    if(output >= 0){
      analogWrite(actuatorPinFwd, output);
      analogWrite(actuatorPinRev, 0);
    }
    else{
      analogWrite(actuatorPinFwd, 0);
      analogWrite(actuatorPinRev, -output);
    }
    
    return output;
  }
  
};

#endif

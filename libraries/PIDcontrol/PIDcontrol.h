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
  int actuatorPin;
  
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
  
  int limit_switch;
  int valve_select;
  
public:

/*  void debugging(){
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
*/
  int getSensor(){
	
    return sensorReading;
  }


  //constructor
  PIDcontrol(int newSensorPin,int newActuatorPin, int newlimit_switch, int newValveSelectPin){
    sensorPin = newSensorPin;
    actuatorPin = newActuatorPin;
	
	limit_switch =  newlimit_switch;
	valve_select = newValveSelectPin;
    
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
  
  int getSetpoint(){
	return setPoint;
  }
  
  int updateOutput(){
	//limit swlitches in series now. need to tweak algorithm
	//hardwire middle point for pots should surfice
	// if limit switch == true and < middle value
	//middle value can be found during calibration
/*	if(digitalRead(limit_switch) == HIGH){
		if(map(analogRead(sensorPin), 0, 1023, 0, 255) > 127){
			setPoint = analogRead(sensorPin);
			setPoint = setPoint-20;
			Serial.println("left? limit switch tripped");
		}
//		else if(map(analogRead(sensorPin), 0, 1023, 0, 255) < 127){
			setPoint = analogRead(sensorPin);
			setPoint = setPoint+20;
			Serial.println("right? limit switch tripped");
		}
	}
	else{//normal operation*/
    //map sensor readings to 0-255 range
		sensorReading = analogRead(sensorPin);
		sensorReading = map(sensorReading, 0, 1023, 0, 255);
    
		error = sensorReading - setPoint;
		//Serial.print("error: ");
		//Serial.println(error);
    
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
		derivative = constrain(derivative,-1024,1024);
    
		//integral constraint for anti-windup
		integral += error;
		integral = constrain(integral, -1024, 1024);
    
		//calculate output
		//adjust fomula to change sensativity to constaints
		output = kp*error - kd*derivative + ki*integral/256;
		output = constrain(output, -235, 235);
    //}
	
    //add dithering regardless of trigger or not
    //output = output + 20*int(sin(float(millis()/2)));
    output = constrain(output, -255, 255);
    
	//here is where we need to set up some way to determine left and right. everything else  works fine
    
	// We may have to give the actuators a "kick" here
	// Possibly set motorspeed to 200 and then back
	// Maybe do something like  void kick in motor class?
	if(output > 5){
//      analogWrite(actuatorPin, output);
//      analogWrite(valve_select, LOW);
      analogWrite(actuatorPin, 200);
      analogWrite(actuatorPin+1, 0);
    }
    else if(output < -5){
 //     analogWrite(actuatorPin, output);
//      analogWrite(valve_select, HIGH);
    
	    analogWrite(actuatorPin+1, 200);
		analogWrite(actuatorPin, 0);
		//if(millis() % 200){
		//Serial.print("Wrote HIGH to ");
		//Serial.println(actuatorPin);}
	}
	else{
		analogWrite(actuatorPin+1,200);
		analogWrite(actuatorPin,0);
	}
    
    return output;
  }
  
};

#endif

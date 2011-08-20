/*
PID control library
calculates the PID output
and writes to output pin
*/
#include<WProgram.h>

//class outputs PID control
//out put constraint to between -255 and 255
class PIDcontrol{
	private:

	int sensorPin;
	int actuatorPin;
	int limit_switch;
	int valve_select;

	int kp;
	int kd;
	int ki;

	int setPoint;

	int sensorReading;
	int Ain, Aout;
	
	int error;
	int derivative;
	int integral;

	int output;
	int dither,ditherFrequency;
	int outOffset;
	int inOffset;

	unsigned long prevTime;
	int prevSensorReading;
	
	bool even;//this is to determine the side of the actuator
	//JULIAN'S FIX for left-right side placement
	//even TRUE means that Ain is low sensor, Aout is high sensor

	public:

	int getSensor(){

		sensorReading = analogRead(sensorPin);
		sensorReading = map(sensorReading, 0, 1023, 0, 255);


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
		ditherFrequency = 200;
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

	void setOutOffset(int o) { outOffset = o; }
	int getOutOffset() { return outOffset; }
	void setInOffset(int i) { inOffset = i; }
	int getInOffset() { return inOffset; }

	/*
	void setConstantP(int newKp){
		kp = newKp;
	}
	void setConstantD(int newKd){
		kd = newKd;
	}
	void setConstantI(int newKi){
		ki = newKi;
	}  
	*/  

	int getPID(int num){
		if(num == 0){return kp;}
		else if(num == 1){return ki;}
		else if(num == 2){return kd;}
		else{return NULL;}
	}
	
	int getAIN(){
		return Ain;
	}
	
	int getAOUT(){
		return Aout;
	}
	
	void calibrated(int newAin, int newAout){
		Ain = newAin;
		Aout = newAout;
		if(Ain < Aout){
			even = true;
		}
		else{
			even = false;
		}
	}
	
	void setSetPoint(int newSetPoint){
		setPoint = newSetPoint;
	}
	
	void setAngle(int newAngle){
		setPoint = map(newAngle,0,50,Ain,Aout);
		}
	
	int getAngle(){
		return map(setPoint,Ain,Aout,0,50);
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
		  //else if(map(analogRead(sensorPin), 0, 1023, 0, 255) < 127){
				setPoint = analogRead(sensorPin);
				setPoint = setPoint+20;
				Serial.println("right? limit switch tripped");
			}
		}
		else{//normal operation*/
		//map sensor readings to 0-255 range
		getSensor();
		error = setPoint - sensorReading;
		if(abs(error) < 2) {
			error = 0;
			integral = 0;
			prevSensorReading = -1;
			//Serial.println("error is 0");
		} 
		else {
			//Serial.println(error);
		}
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

		if(prevSensorReading != sensorReading) {
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
		//SSerial.println(output);
		output = constrain(output, -255, 255);
		//}
		

		int realOutput = output;
		realOutput = constrain(realOutput, -255, 255);
		//Serial.println(realOutput);
		//here is where we need to set up some way to determine left and right. everything else  works fine

		// We map the output to 120 to 255 here because the
		// Valves won't open enough below 120 duty cycle
		if(realOutput > 0){
			analogWrite(actuatorPin, map(realOutput, 0, 255, 120, 255));
			if(even){
				digitalWrite(valve_select, HIGH);
			}
			else{
				digitalWrite(valve_select, LOW);
			}
		}
		else if(realOutput < 0){
			analogWrite(actuatorPin, -map(realOutput, -255, 0, -255, -120));
			if(even){
				digitalWrite(valve_select, LOW);
			}
			else{
				digitalWrite(valve_select, HIGH);
			}

		}
		else{
			/*dither = int(sin(float(millis()/1000.0*2.0*PI*ditherFrequency)));
			if(dither > 0){
				digitalWrite(actuatorPin,HIGH);
				digitalWrite(valve_select,HIGH);
			}
			else{
				digitalWrite(actuatorPin,LOW);
				digitalWrite(valve_select,HIGH);
			}*/
			analogWrite(actuatorPin,0);
		}

		return realOutput;
	}

};

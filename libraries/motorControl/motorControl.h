class motorControl {
	public:
		motorControl(int NewMotorPin){
			hmotorPin = NewMotorPin;
			hmotorMax=200;
			off();
		}
		//zero everything
		void off(){
			for(int i = 0; i < 5; i++){
				actuatorX[i]=0;
				actuatorZ[i]=0;
			}
			aSum = 0;
			motorScaled=0;
			digitalWrite(hmotorPin,0);
		}
		//new value for an X actuator
		void updateAX(int nActuator, int aValueX){
			actuatorX[nActuator] = aValueX;
		}
		//new value for a Z actuator
		void updateAZ(int nActuator, int aValueX){
			actuatorZ[nActuator] = aValueX;
		}
		//change the max value of the motor
		void maxSet(int newMax){
			hmotorMax = newMax;
		}
		//update the motor output
		void updateMotor(){
			aSum = 0;
			for(int i = 0; i < 5; i++){
				aSum += actuatorX[i];
				aSum += actuatorZ[i];
			}
			//Serial.print(aSum);
			//Serial.print(" ");
			//Serial.print(hmotorMax*2);
			//Serial.print(" ");
			//Serial.print(hmotorMax*2*(aSum/2550));
			//Serial.print(" ");
			//motorScaled = (hmotorMax*2*aSum)/2550;//2550 or something else and linear?
			//Serial.println(motorScaled);
			//Serial.println();
			if(abs(aSum) > 20)
			{
				motorScaled=80;
			}
			else
			{
				motorScaled=0;
			}
			if(abs(aSum) > 60){motorScaled=100;}
			if(abs(aSum) > 100) {motorScaled=120;}
			
			analogWrite(hmotorPin,abs(motorScaled));
		}
		
		int getSpeed(){
			return motorScaled;
		}
		
		int getMax(){
			return hmotorMax;
		}

	private:
		int actuatorX[5];//values between 0 and 255
		int	actuatorZ[5];//values between 0 and 255
		int motorScaled;//motor value compared to active actuators
		int hmotorMax;//max value for motor
		int hmotorPin;//what pin am i writing to
		int aSum;//sum of all the < 255 values
};

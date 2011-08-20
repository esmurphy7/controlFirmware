//to organize A_messagePump better
//Ian split the functions into different files
//
// Blocks until all five horizontal limit switches have
// been tripped. If a switch is tripped the actuator is
// turned off and the trip is registered. When all five
// switches have been tripped the function returns
void delayForLimitSwitches() {
  boolean tripped[] = {false, false, false, false, false};
  // Loop until all limit switches are tripped
  while(1) {
    int num_tripped = 0;
    // Loop through each switch
    for(int i = 0; i<5; i++) {
      // If the switch has been tripped turn off the actuator
      //    and store the sensor values
      if(digitalRead(limit_switchX[i])) {
        tripped[i] = true;
      }
      // Sum the number of switches tripped while checking
      if(tripped[i] == true) {
        num_tripped++;
      }
    }
    // If all five limit switches were tripped break out of the loop
    if(num_tripped == 5) {
      break;
    }
    delay(10);
  }
}

void calibrate(){
  //extend then retract all at once until

  int Ain[5];
  int Aout[5];
  int cur_pid[5];
  
  // Turn on the motor
  analogWrite(motorPin,130);
  
  // Turn on actuators to HIGH side
  for(int i=0; i<5; i++){
    digitalWrite(valveSelectX[i],HIGH);
    analogWrite(actuatorPinX[i],255);
  }
  delay(1000);
  //delayForLimitSwitches();
  
  // Turn off all actuators and get thier sensor values
  for(int i=0; i<5; i++){  
    analogWrite(actuatorPinX[i],0);
    Aout[i] = PIDcontrollerX[i].getSensor();
  }
  
  Serial.print("High value is ");
  for(int i=0; i<5; i++){  
    Serial.println(Aout[i],DEC);
  }
  
  // Turn on actuators to LOW side
  for(int i=0; i<5; i++){    
    digitalWrite(valveSelectX[i],LOW);
    analogWrite(actuatorPinX[i],250);
  }
  delay(1000);
  //delayForLimitSwitches();
  
  for(int i=0; i<5; i++){  
    analogWrite(actuatorPinX[i],0);
    Ain[i] = PIDcontrollerX[i].getSensor();
  }
  analogWrite(motorPin,0);
  Serial.print("Low value is ");
  for(int i=0; i<5; i++){  
    Serial.println(Ain[i],DEC);
  }
  for(int i=0; i<5; i++){
    PIDcontrollerX[i].calibrated(Ain[i],Aout[i]);
    cur_pid[i] = (Aout[i] + Ain[i])/2 + 2;//little bit of curve
  }
}

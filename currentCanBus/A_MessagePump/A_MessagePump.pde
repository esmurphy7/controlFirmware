// Command pump for slave Arduino
// Reads commands and responds as required
#include "interface.h"
#include "MCP2515.h"
#include <SPI.h>
#include "PIDcontrolBOA.h"
#include  "boashield_pins.h"
#include <motorControl.h>


#define CS_PIN 53

Interface interface(CS_PIN);

/*X is for horizontal
  Z is for vertical*/
 
#define  ditherFreq  200

//THESE PINS WERE CHECKED ON JULY 20 2011
//analog pins for pot reading
// Pins defined in boashield_pins.h
const int sensorPinX[5] = {HORZ_POS_SENSOR_1, 
                          HORZ_POS_SENSOR_2, 
                          HORZ_POS_SENSOR_3, 
                          HORZ_POS_SENSOR_4, 
                          HORZ_POS_SENSOR_5};

const int sensorPinZ[5] = { VERT_POS_SENSOR_1, 
                          VERT_POS_SENSOR_2,
                          VERT_POS_SENSOR_3,
                          VERT_POS_SENSOR_4,
                          VERT_POS_SENSOR_5};

//PID signal pins
const int actuatorPinX[5] = {HORZ_ACTUATOR_1, 
                            HORZ_ACTUATOR_2, 
                            HORZ_ACTUATOR_3, 
                            HORZ_ACTUATOR_4, 
                            HORZ_ACTUATOR_5};

const int actuatorPinZ[5] = {VERT_ACTUATOR_1, 
                            VERT_ACTUATOR_2, 
                            VERT_ACTUATOR_3, 
                            VERT_ACTUATOR_4, 
                            VERT_ACTUATOR_5};

//Valve select pins because we don't have enough PWM
const int valveSelectX[5] = {HORZ_ACTUATOR_CTRL_1, 
                            HORZ_ACTUATOR_CTRL_2, 
                            HORZ_ACTUATOR_CTRL_3, 
                            HORZ_ACTUATOR_CTRL_4, 
                            HORZ_ACTUATOR_CTRL_5};

const int valveSelectZ[5] = {VERT_ACTUATOR_CTRL_1, 
                            VERT_ACTUATOR_CTRL_2, 
                            VERT_ACTUATOR_CTRL_3, 
                            VERT_ACTUATOR_CTRL_4, 
                            VERT_ACTUATOR_CTRL_5};

//Dither will be fed into all unselected valves
const int ditherPin = DITHER;

//motor write speed
const int motorPin = MOTOR_CONTROL;

motorControl	motorController(motorPin);

/*limit switches in series. will determine L/R through math*/
const int limit_switchX[5] = {HORZ_LIMIT_SWITCH_1, 
                             HORZ_LIMIT_SWITCH_2,
                             HORZ_LIMIT_SWITCH_3, 
                             HORZ_LIMIT_SWITCH_4, 
                             HORZ_LIMIT_SWITCH_5};
                             
const int limit_switchZ[5] = {VERT_LIMIT_SWITCH_1, 
                             VERT_LIMIT_SWITCH_2, 
                             VERT_LIMIT_SWITCH_3, 
                             VERT_LIMIT_SWITCH_4, 
                             VERT_LIMIT_SWITCH_5};

//have to check battery voltage
const int batPin = BAT_LEVEL_24V;

//arduinos need to kill

//PID controllers
PIDcontrol PIDcontrollerX[5] = {
  PIDcontrol( sensorPinX[0], actuatorPinX[0], limit_switchX[0], valveSelectX[0]),
  PIDcontrol( sensorPinX[1], actuatorPinX[1], limit_switchX[1], valveSelectX[1]),
  PIDcontrol( sensorPinX[2], actuatorPinX[2], limit_switchX[2], valveSelectX[2]),
  PIDcontrol( sensorPinX[3], actuatorPinX[3], limit_switchX[3], valveSelectX[3]),
  PIDcontrol( sensorPinX[4], actuatorPinX[4], limit_switchX[4], valveSelectX[4])
};
///*
PIDcontrol PIDcontrollerZ[5] = {
  PIDcontrol( sensorPinZ[0], actuatorPinZ[0], limit_switchZ[0], valveSelectZ[0]),
  PIDcontrol( sensorPinZ[1], actuatorPinZ[1], limit_switchZ[1], valveSelectZ[1]),
  PIDcontrol( sensorPinZ[2], actuatorPinZ[2], limit_switchZ[2], valveSelectZ[2]),
  PIDcontrol( sensorPinZ[3], actuatorPinZ[3], limit_switchZ[3], valveSelectZ[3]),
  PIDcontrol( sensorPinZ[4], actuatorPinZ[4], limit_switchZ[4], valveSelectZ[4])
};
//*/

//defaul values for stuff
//motor
int motorValue = 127;

//PID calibrate
boolean pidCalibrated = false;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  int type = SLAVE;
  
  interface.init(type);
  delay(10);
  
  pinMode(motorPin, OUTPUT);
  
  //make sure actuators are off and set pin modes
  
  pinMode(ditherPin, OUTPUT);
//  digitalWrite(ditherPin,DITHERPWM);

  for(int i=0;i<5;i++){
    pinMode(actuatorPinX[i], OUTPUT);
    analogWrite(actuatorPinX[i], 0);
    
    pinMode(valveSelectX[i], OUTPUT);
   
    pinMode(actuatorPinZ[i], OUTPUT);
    analogWrite(actuatorPinZ[i], 0);
    
    pinMode(valveSelectZ[i], OUTPUT);
  }
  
  //make set point current value(so it dosen't move)
  for(int i=0;i<5;i++){
    PIDcontrollerX[i].setSetPoint(analogRead(sensorPinX[i]));
    PIDcontrollerZ[i].setSetPoint(analogRead(sensorPinZ[i]));
  }
  checkBat();
  BcheckTIME = millis();
}//setup end

Frame command;

void ditherF(){
  int Dout = int(sin(float(millis()/1000.0*2.0*PI*ditherFreq)));
  if(Dout > 0){
    digitalWrite(ditherPin,HIGH);
  }
  else{
    digitalWrite(ditherPin,LOW);
  }
  //analogWrite(ditherPin,127);
}

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
  delayForLimitSwitches();
  
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
  delayForLimitSwitches();
  
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

int checkBat(){
  int batIN = analogRead(BAT_LEVEL_24V);
  int batREAL = map(batIN,0,1023,0,25000);
  if(batREAL < batDIE){
    //whatever we do to trigger kill switches
  }
  return batREAL;
}

unsigned long BcheckTIME;
//remember, it's in mV
int batDIE=21000;

void loop(){
  command = interface.getMessage();
  switch(command.data[0]){
    
    case CMD_NONE:
      //Serial.println("CMD_NONE");
      break;
      
    case CMD_SET_HORIZ:
        //Serial.println("CMD_SET_HORIZ");
      for (int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setSetPoint(0|command.data[i]);
      }
      interface.sendState(STATE_HORIZ,PIDcontrollerX[0].getSensor(),PIDcontrollerX[1].getSensor(),PIDcontrollerX[2].getSensor(),PIDcontrollerX[3].getSensor(),PIDcontrollerX[4].getSensor(),PAD);
/*      Serial.print(PIDcontrollerX[0].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[1].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[2].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[3].getSensor(),DEC);
      Serial.print(" ");
      Serial.print(PIDcontrollerX[4].getSensor(),DEC);
      Serial.println();*/
      break;
      
    case CMD_SET_VERT:
      //Serial.println("CMD_SET_VERT");
      for (int i=1;i<=5;i++){
        PIDcontrollerZ[i-1].setSetPoint(0|command.data[i]);
      }
      interface.sendState(STATE_VERT,PIDcontrollerZ[0].getSensor(),PIDcontrollerZ[1].getSensor(),PIDcontrollerZ[2].getSensor(),PIDcontrollerZ[3].getSensor(),PIDcontrollerZ[4].getSensor(),PAD);
      break;
      
    case CMD_SET_ANGLEX:
      for (int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setAngle(0|command.data[i]);
      }
      interface.sendState(STATE_ANGLEX,PIDcontrollerX[0].getAngle(),PIDcontrollerX[1].getAngle(),PIDcontrollerX[2].getAngle(),PIDcontrollerX[3].getAngle(),PIDcontrollerX[4].getAngle(),PAD);
      break;
      
    case CMD_SET_ANGLEZ:
      for (int i=1;i<=5;i++){
        PIDcontrollerZ[i-1].setAngle(0|command.data[i]);
      }
      interface.sendState(STATE_ANGLEX,PIDcontrollerZ[0].getAngle(),PIDcontrollerZ[1].getAngle(),PIDcontrollerZ[2].getAngle(),PIDcontrollerZ[3].getAngle(),PIDcontrollerZ[4].getAngle(),PAD);
      break;
      
    case CMD_PRINT_H:
      interface.sendState(STATE_HORIZ,PIDcontrollerX[0].getSensor(),PIDcontrollerX[1].getSensor(),PIDcontrollerX[2].getSensor(),PIDcontrollerX[3].getSensor(),PIDcontrollerX[4].getSensor(),PAD);
      break;
      
    case CMD_PRINT_V:
      interface.sendState(STATE_VERT,PIDcontrollerZ[0].getSensor(),PIDcontrollerZ[1].getSensor(),PIDcontrollerZ[2].getSensor(),PIDcontrollerZ[3].getSensor(),PIDcontrollerZ[4].getSensor(),PAD);
      break;
      
    case CMD_GET_ANGLEX:
      interface.sendState(STATE_ANGLEX,PIDcontrollerX[0].getAngle(),PIDcontrollerX[1].getAngle(),PIDcontrollerX[2].getAngle(),PIDcontrollerX[3].getAngle(),PIDcontrollerX[4].getAngle(),PAD);
      break;
      
    case CMD_GET_ANGLEZ:
       interface.sendState(STATE_ANGLEX,PIDcontrollerZ[0].getAngle(),PIDcontrollerZ[1].getAngle(),PIDcontrollerZ[2].getAngle(),PIDcontrollerZ[3].getAngle(),PIDcontrollerZ[4].getAngle(),PAD);
       break;
       
    case CMD_SET_MOTOR:
      Serial.println("CMD_SET_MOTOR");
      motorController.maxSet(command.data[1]);
      interface.sendState(STATE_MOTOR,command.data[1],motorController.getSpeed(),PAD,PAD,PAD,PAD);
      break;
      
    case CMD_PRINT_MOT:
      interface.sendState(STATE_MOTOR,motorController.getMax(),motorController.getSpeed(),PAD,PAD,PAD,PAD);
      break;
      
    case CMD_SEND_BAT:
      //Serial.println("CMD_SEND_BAT");
      interface.sendState(STATE_BAT,checkBat(),batDIE,PAD,PAD,PAD,PAD);
      break;
      
    case CMD_SET_DIE:// battery kill level
      batDIE = command.data[1];
      break;
      
    case CMD_SET_PID:
      Serial.println("CMD_SET_PID");
      if(command.data[1] > 4) {
        for(int i = 0; i<5; i++) {
          PIDcontrollerX[i].setConstants(0|command.data[2],0|command.data[3],0|command.data[4]);
        }
      }
      else {
        PIDcontrollerX[command.data[1]].setConstants(0|command.data[2],0|command.data[3],0|command.data[4]);
  //    PIDcontrollerZ[i-1].setConstants(0|command.data[1],0|command.data[2],0|command.data[3]);
        interface.sendState(STATE_PID,command.data[2], command.data[3], command.data[4], PAD,PAD,PAD);
      }
      break;
      
    case CMD_GET_PID:
    //same problem as PID SET
      if(command.data[1] > 4) {	  
	    for(int i=0;i<4;i++){
	  	  interface.sendState(STATE_PID, i,PIDcontrollerX[i].getPID(0), PIDcontrollerX[i].getPID(1), PIDcontrollerX[i].getPID(2),PAD,PAD);
	    }
	  }
	  else{
		interface.sendState(STATE_PID, command.data[1]],PIDcontrollerX[command.data[1]]].getPID(0), PIDcontrollerX[command.data[1]]].getPID(1), PIDcontrollerX[command.data[1]]].getPID(2),PAD,PAD);
      }
	  break;
      
    case CMD_PID_CAL:
     calibrate();     
     pidCalibrated = true;
     interface.sendState(STATE_PID_CAL,PAD,PAD,PAD,PAD,PAD,PAD);
     break;
     
    case CMD_SET_SENSR://CRAP about limit switches.
      //Serial.println("CMD_SET_SENSR");
      interface.sendState(STATE_AIN,PIDcontrollerX[0].getAIN(),PIDcontrollerX[1].getAIN(),PIDcontrollerX[2].getAIN(),PIDcontrollerX[3].getAIN(),PIDcontrollerX[4].getAIN(),PAD);
      delay(100);
      interface.sendState(STATE_AOUT,PIDcontrollerX[0].getAOUT(),PIDcontrollerX[1].getAOUT(),PIDcontrollerX[2].getAOUT(),PIDcontrollerX[3].getAOUT(),PIDcontrollerX[4].getAOUT(),PAD);
      break;
      
    case CMD_BROADCAST:
      //Serial.println("CMD_BROADCAST");
      interface.broadcast();
      break;
      
    case CMD_STOP:
      //should do some relay killing
      //all but arduino off
      //Serial.println("CMD_STOP");
      motorController.off();
      interface.sendState(STATE_STOPPED,PAD,PAD,PAD,PAD,PAD,PAD);
      break;
      
    case CMD_START:
      //Serial.println("CMD_START");
      motorController.setON();
      interface.sendState(STATE_GOING,PAD,PAD,PAD,PAD,PAD,PAD);      
      break;
      
    case CMD_STANDBY:
      //Serial.println("CMD_STANDBY");
      motorController.off();
      interface.sendState(STATE_STANDBY,PAD,PAD,PAD,PAD,PAD,PAD);
      break;
      
    case CMD_DIE:
      motorController.off();
      //ack die and power down
      break;
      
    case CMD_DATA_DUMP://master command. not slave. really need here?
      //send back what needs to be sent
      break;
  }//end of switch
/*++++++++++++++++++++output++++++++++++++++++++++++++++++++*/
  if(pidCalibrated == true){
    //use PID to make actuator adjustments
    for(int i=0;i<5;i++){
      outputX[i]=PIDcontrollerX[i].updateOutput();
      motorController.updateAX(i,outputX[i]);
     // /*
//      PIDcontrollerZ[i].updateOutput();
    //  */
    }
    motorController.updateMotor();
  }
  else{
    for(int i=0;i<5;i++){
      PIDcontrollerX[i].setSetPoint(PIDcontrollerX[i].getSensor());
      PIDcontrollerX[i].updateOutput();
     // /*
//      PIDcontrollerZ[i].setSetPoint(PIDcontrollerZ[i].getSensor());
//      PIDcontrollerZ[i].updateOutput();
    //  */
      motorController.updateMotor();
    }
  }
/*++++++++++++++++++++output++++++++++++++++++++++++++++++++*/
  if((millis()-BcheckTIME)>50000){
    checkBat();
	BcheckTIME=millis();
  }

}

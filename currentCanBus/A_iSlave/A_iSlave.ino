// Command pump for slave Arduino
// Reads commands and responds as required

//In the version made around august 16, no PIDZ
//to still allow for vertical movement

//Open Loop cases beginning to be added August 18th
//ideally what Ian want's to have done is a solid CLOSED LOOP, OPEN LOOP, OFF, DYING states of operatoin
//little awkward just as we head to burning man but oh well
//this is the 20 hour mark for ian on his marathon coding before packup
//with an update for Arduino 1.0 and not sure if will work properly
#include <SPI.h>
#include "boashield_pins.h"
#include "PinInit.h"
#include "MCP2515.h"
#include "interface.h"

#define CS_PIN 53

Interface interface(CS_PIN);

#include "PIDcontrolBOA.h"
#include <motorControl.h>
#include "BatteryInfo.h"

/*X is for horizontal
  Z is for vertical*/
 
#define  ditherFreq  200

motorControl	motorController(motorPin);

//PID controllers
PIDcontrol PIDcontrollerX[5] = {
  PIDcontrol( sensorPinX[0], actuatorPinX[0], limit_switchX[0], valveSelectX[0]),
  PIDcontrol( sensorPinX[1], actuatorPinX[1], limit_switchX[1], valveSelectX[1]),
  PIDcontrol( sensorPinX[2], actuatorPinX[2], limit_switchX[2], valveSelectX[2]),
  PIDcontrol( sensorPinX[3], actuatorPinX[3], limit_switchX[3], valveSelectX[3]),
  PIDcontrol( sensorPinX[4], actuatorPinX[4], limit_switchX[4], valveSelectX[4])
};
///*Z controller should always be initialized. less pin-based problems that way
PIDcontrol PIDcontrollerZ[5] = {
  PIDcontrol( sensorPinZ[0], actuatorPinZ[0], limit_switchZ[0], valveSelectZ[0]),
  PIDcontrol( sensorPinZ[1], actuatorPinZ[1], limit_switchZ[1], valveSelectZ[1]),
  PIDcontrol( sensorPinZ[2], actuatorPinZ[2], limit_switchZ[2], valveSelectZ[2]),
  PIDcontrol( sensorPinZ[3], actuatorPinZ[3], limit_switchZ[3], valveSelectZ[3]),
  PIDcontrol( sensorPinZ[4], actuatorPinZ[4], limit_switchZ[4], valveSelectZ[4])
};
//*/
#include "Calibration.h"

//bools related to running mode
boolean pidCalibrated = false;//needed to work
boolean OpenLoopMode = false;
boolean ClosedLoopMode = false;
Frame command;


//still not entirely sure how this will affect the valve output
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
//they want vertical actuation so vertical actuation I will give them
//boolean flags and timers will do the trick
//this also applies to the open loop case we want as backup
boolean openVertB[5]={false,false,false,false,false};
unsigned long openVertT[5]={0,0,0,0,0};

boolean openHorizB[5]={false,false,false,false,false};
unsigned long openHorizT[5]={0,0,0,0,0};

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
  //the solution to our probable power on initialization  
  interface.CANinit();
  interface.sendState(STATE_INITING,PAD,PAD,PAD,PAD,PAD,PAD);
  interface.init(type);
  delay(10);
  
  pinMode(motorPin, OUTPUT);
  
  //make sure actuators are off and set pin modes
  
  pinMode(ditherPin, OUTPUT);
//  digitalWrite(ditherPin,DITHERPWM);
  Serial.println(millis());
  for(int i=0;i<5;i++){
    pinMode(actuatorPinX[i], OUTPUT);
    analogWrite(actuatorPinX[i], 0);
    
    pinMode(valveSelectX[i], OUTPUT);
   
    pinMode(actuatorPinZ[i], OUTPUT);
    analogWrite(actuatorPinZ[i], 0);
    
    pinMode(valveSelectZ[i], OUTPUT);
  }
  Serial.println(millis());
  //make set point current value(so it dosen't move)
  for(int i=0;i<5;i++){
    PIDcontrollerX[i].setSetPoint(analogRead(sensorPinX[i]));
    PIDcontrollerZ[i].setSetPoint(analogRead(sensorPinZ[i]));
  }
  Serial.println(millis());
  checkBat();
  BcheckTIME = millis();
  Serial.println("Entering Loop");
}//setup end

void loop(){
  int outputX[5];
  command = interface.getMessage();
  if(command.data[0] != CMD_NONE){
    for(int i=0;i<7;i++){
      Serial.print(command.data[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }
  switch(command.data[0]){
    
    case CMD_NONE:
      //Serial.println("CMD_NONE");
      break;
    
    //gives the PID controller a value between 0 and 255
  //this relates to the sensor readings  
    case CMD_SET_HORIZ:
      Serial.println("CMD_SET_HORIZ");
      Serial.print("horizontal start ");
      Serial.println(millis());
      for (int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setSetPoint(0|command.data[i]);
      }
      Serial.print("horizontal done ");
      Serial.print(millis());
      interface.sendState(STATE_HORIZ,PIDcontrollerX[0].getSensor(),PIDcontrollerX[1].getSensor(),PIDcontrollerX[2].getSensor(),PIDcontrollerX[3].getSensor(),PIDcontrollerX[4].getSensor(),PAD);
      Serial.print(" ");
      Serial.println(millis());
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
    
    //currently only operating in open loop with timers and flags
  //will eventually accept a value between 0 and 255 to reach feedback sensor?  
    case CMD_SET_VERT:
      Serial.println("CMD_SET_VERT");
 //     for (int i=1;i<=5;i++){
   //     PIDcontrollerZ[i-1].setSetPoint(0|command.data[i]);
     // }
      //interface.sendState(STATE_VERT,PIDcontrollerZ[0].getSensor(),PIDcontrollerZ[1].getSensor(),PIDcontrollerZ[2].getSensor(),PIDcontrollerZ[3].getSensor(),PIDcontrollerZ[4].getSensor(),PAD);
      
      //the open loop case
      //true or false
      for(int i = 1; i<=5; i++){
        //turning off for sure case
        if(command.data[i]==0){
          openVertB[i-1]=false;
          analogWrite(actuatorPinZ[i-1],0);
        }
        //the going in case
        else if(command.data[i]<128){
          openVertB[i-1]=true;
          digitalWrite(valveSelectZ[i-1],LOW);
          analogWrite(actuatorPinZ[i-1],200);
          openVertT[i-1]=millis();
        }
        //the going out case
        else{
          digitalWrite(valveSelectZ[i-1],HIGH);
          analogWrite(actuatorPinZ[i-1],200);
          openVertT[i-1]=millis();
        }
      }
      interface.sendState(STATE_VERT,openVertB[0],openVertB[1],openVertB[2],openVertB[3],openVertB[4],PAD);
      break;
    
    //this is the more ideal case for shifting information down the snake
  //an angle which is then translated into pot value by PID controller  
    case CMD_SET_ANGLEX:
      Serial.println("CMD_SET_ANGLEX");
      for (int i=1;i<=5;i++){
        PIDcontrollerX[i-1].setAngle(0|command.data[i]);
      }
      interface.sendState(STATE_ANGLEX,PIDcontrollerX[0].getAngle(),PIDcontrollerX[1].getAngle(),PIDcontrollerX[2].getAngle(),PIDcontrollerX[3].getAngle(),PIDcontrollerX[4].getAngle(),PAD);
      break;
    
    //same idea as anglex, only currently operating in open loop
  //open loop case just has it go open and then closed for a moment  
    case CMD_SET_ANGLEZ:
      Serial.println("CMD_SET_ANGLEZ");
//      for (int i=1;i<=5;i++){
  //      PIDcontrollerZ[i-1].setAngle(0|command.data[i]);
    //  }
      //interface.sendState(STATE_ANGLEZ,PIDcontrollerZ[0].getAngle(),PIDcontrollerZ[1].getAngle(),PIDcontrollerZ[2].getAngle(),PIDcontrollerZ[3].getAngle(),PIDcontrollerZ[4].getAngle(),PAD);
      //the open loop case
      //true or false
      for(int i = 1; i<=5; i++){
        //turning off for sure case
        if(command.data[i]==0){
          openVertB[i-1]=false;
          analogWrite(actuatorPinZ[i-1],0);
        }
        //the going in case
        else if(command.data[i]<128){
          openVertB[i-1]=true;
          digitalWrite(valveSelectZ[i-1],LOW);
          analogWrite(actuatorPinZ[i-1],200);
          openVertT[i-1]=millis();
        }
        //the going out case
        else{
          digitalWrite(valveSelectZ[i-1],HIGH);
          analogWrite(actuatorPinZ[i-1],200);
          openVertT[i-1]=millis();
        }
      }
      interface.sendState(STATE_ANGLEZ,openVertB[0],openVertB[1],openVertB[2],openVertB[3],openVertB[4],PAD);
      break;
    
    //prints out the current horizontal pot value  
    case CMD_PRINT_H:
      Serial.println("CMD_PRINT_H");
      interface.sendState(STATE_HORIZ,PIDcontrollerX[0].getSensor(),PIDcontrollerX[1].getSensor(),PIDcontrollerX[2].getSensor(),PIDcontrollerX[3].getSensor(),PIDcontrollerX[4].getSensor(),PAD);
      break;
    
    //will print out the current vertical pot value
  //currently only shows if the vertical actuator should be on or not  
    case CMD_PRINT_V:
      Serial.println("CMD_PRINT_V");
      //interface.sendState(STATE_VERT,PIDcontrollerZ[0].getSensor(),PIDcontrollerZ[1].getSensor(),PIDcontrollerZ[2].getSensor(),PIDcontrollerZ[3].getSensor(),PIDcontrollerZ[4].getSensor(),PAD);
       interface.sendState(STATE_VERT,openVertB[0],openVertB[1],openVertB[2],openVertB[3],openVertB[4],PAD);     
      break;
    
    //will return the current angle the joint is suppose to be making
  //still under the impression that max angle is +/- 25 degrees   
    case CMD_GET_ANGLEX:
      Serial.println("CMD GET ANGLEX");
      interface.sendState(STATE_ANGLEX,PIDcontrollerX[0].getAngle(),PIDcontrollerX[1].getAngle(),PIDcontrollerX[2].getAngle(),PIDcontrollerX[3].getAngle(),PIDcontrollerX[4].getAngle(),PAD);
      break;
    
    //will eventually do the and thing as angleX
  //currently only shows if vertical actuator should be on or not
    case CMD_GET_ANGLEZ:
    Serial.println("CMD GET ANGLEZ");
       //interface.sendState(STATE_ANGLEX,PIDcontrollerZ[0].getAngle(),PIDcontrollerZ[1].getAngle(),PIDcontrollerZ[2].getAngle(),PIDcontrollerZ[3].getAngle(),PIDcontrollerZ[4].getAngle(),PAD);
      interface.sendState(STATE_ANGLEZ,openVertB[0],openVertB[1],openVertB[2],openVertB[3],openVertB[4],PAD);       break;
      break;
    
    //changes the max value the motor can reach.
 //motor scaling algorithm is untested at this point though
//returns the max value that was just passed through and current speed 
    case CMD_SET_MOTOR:
      Serial.println("CMD_SET_MOTOR");
      motorController.maxSet(command.data[1]);
      interface.sendState(STATE_MOTOR,command.data[1],motorController.getSpeed(),PAD,PAD,PAD,PAD);
      break;
    
    //status command that prints the current max motor value and current speed
    case CMD_PRINT_MOT:
      Serial.println("CMD PRINT MOTOR");
      interface.sendState(STATE_MOTOR,motorController.getMax(),motorController.getSpeed(),PAD,PAD,PAD,PAD);
      break;
    
    //send current batter information  
    case CMD_SEND_BAT:
      Serial.println("CMD_SEND_BAT");
      interface.sendState(STATE_BAT,checkBat(),batDIE,PAD,PAD,PAD,PAD);
      break;
    
    //sets the battery die level from the default to something new  
    case CMD_SET_DIE:// battery kill level
      Serial.println("CMD SET DIE");
      batDIE = command.data[1];
      break;
    
    //sets the new PID values for either an individual controller or entire module  
    case CMD_SET_PID:
      Serial.println("CMD_SET_PID");
      if(command.data[1] > 4) {
        for(int i = 0; i<5; i++) {
          PIDcontrollerX[i].setConstants(0|command.data[2],0|command.data[3],0|command.data[4]);
        }
      }
      else {
        PIDcontrollerX[command.data[1]].setConstants(0|command.data[2],0|command.data[3],0|command.data[4]);
        interface.sendState(STATE_PID,command.data[2], command.data[3], command.data[4], PAD,PAD,PAD);
      }
      break;
    
    //sends us the current PID values for the section or individual PID controller  
    case CMD_GET_PID:
      Serial.println("CMD GET PID");
    //same problem as PID SET
      if(command.data[1] > 4) {	  
	    for(int i=0;i<3;i++){
	      interface.sendState(STATE_PID,PIDcontrollerX[0].getPID(i), PIDcontrollerX[1].getPID(i), PIDcontrollerX[2].getPID(i),PIDcontrollerX[3].getPID(i),PIDcontrollerX[4].getPID(i),PAD);
	    }
	  }
	  else{
		interface.sendState(STATE_PID, command.data[1],PIDcontrollerX[command.data[1]].getPID(0), PIDcontrollerX[command.data[1]].getPID(1), PIDcontrollerX[command.data[1]].getPID(2),PAD,PAD);
          }
	  break;
    
    //before the snake can move properly or at all for that matter
  //this musts be run. gives us Ain and Aout values  
  //Ian has set it so that open has to be false before calibrated can be turned on
    case CMD_PID_CAL:
      Serial.println("PID CAL");
      if(OpenLoopMode==false){
        Serial.println(millis());
        calibrate();
        Serial.println(millis());
        pidCalibrated = true;
        interface.sendState(STATE_PID_CAL,true,PAD,PAD,PAD,PAD,PAD);
      }
      else{
        interface.sendState(STATE_PID_CAL,false,PAD,PAD,PAD,PAD,PAD);
      }
      break;
    
    //was being used for something else
   //now being used to send back Ain and Aout values 
    case CMD_SET_SENSR://CRAP about limit switches.
      Serial.println("CMD_SET_SENSR");
      interface.sendState(STATE_AIN,PIDcontrollerX[0].getAIN(),PIDcontrollerX[1].getAIN(),PIDcontrollerX[2].getAIN(),PIDcontrollerX[3].getAIN(),PIDcontrollerX[4].getAIN(),PAD);
      delay(100);
      interface.sendState(STATE_AOUT,PIDcontrollerX[0].getAOUT(),PIDcontrollerX[1].getAOUT(),PIDcontrollerX[2].getAOUT(),PIDcontrollerX[3].getAOUT(),PIDcontrollerX[4].getAOUT(),PAD);
      break;
    
    //sends "I am alive"  
    case CMD_BROADCAST:
      Serial.println("CMD_BROADCAST");
      interface.broadcast();
      break;
/********************************************************************/
/********The Four, now 3 and maybe changing more, Great States*******/
    case CMD_STOP:
      //should do some relay killing
      //all but arduino off
      Serial.println("CMD_STOP");
      analogWrite(ditherPin,0);
      motorController.off();
      //this means we'll enter the other aka off state
      ClosedLoopMode = false;
      OpenLoopMode = false;
      
      interface.sendState(STATE_STOPPED,PAD,PAD,PAD,PAD,PAD,PAD);
      break;
      
    case CMD_START:
      Serial.println("CMD_START");
      motorController.setON();
      if(command.data[1]==0x11){
        OpenLoopMode = true;
        ClosedLoopMode = false;
      }
      if(command.data[1]==0x22){
        ClosedLoopMode = true;
        OpenLoopMode = false;
      }
      //there is no else case cause nothing will happen otherwise
      interface.sendState(STATE_GOING,ClosedLoopMode,OpenLoopMode,PAD,PAD,PAD,PAD);      
      break;
      
/*  standby and stop are essentially the same things so says james  case CMD_STANDBY:
      //Serial.println("CMD_STANDBY");
      motorController.off();
      interface.sendState(STATE_STANDBY,PAD,PAD,PAD,PAD,PAD,PAD);
      break;*/
      
    case CMD_DIE:
      Serial.println("CMD DIE");
      motorController.off();
      //ack die and power down
      break;
/**********************The Fourish Great States*************************/
/********************************************************************/

    //not sure what to do with this one anymore. lots more status information than before  
    case CMD_DATA_DUMP://master command. not slave. really need here?
      Serial.println("CMD USELESS DUMP");
      //send back what needs to be sent
      break;
    
    //because powering everything on and off manually won't always be an option
    case CMD_REINIT:
      Serial.println("CMD REINITIALIZATION");
    //this will happen if a new module turns on or resets
      Serial.println(millis());
      for(int i=0; i<5; i++){
        PIDcontrollerX[i].setSetPoint(PIDcontrollerX[i].getSensor());
        PIDcontrollerX[i].updateOutput();
      }
      motorController.off();
      motorController.updateMotor();
      Serial.print("starting another initialize command");
      Serial.println(millis());
      interface.init(SLAVE);
      Serial.print("ending another initialize command");
      Serial.println(millis());
      break;
      
    case CMD_OPEN_HORIZ:
      Serial.println("CMD OPEN LOOP HORIZONTAL");
      for(int i = 1; i<=5; i++){
        //turning off for sure case
        if(command.data[i]==0){
          openHorizB[i-1]=false;
          analogWrite(actuatorPinX[i-1],0);
        }
        //the going in case
        else if(command.data[i]<128){
          openHorizB[i-1]=true;
          digitalWrite(valveSelectX[i-1],LOW);
          analogWrite(actuatorPinX[i-1],200);
          openHorizT[i-1]=millis();
          motorController.updateAX(i,200);
        }
        //the going out case
        else{
          digitalWrite(valveSelectX[i-1],HIGH);
          analogWrite(actuatorPinX[i-1],200);
          openHorizT[i-1]=millis();
          motorController.updateAX(i,200);
        }
      }
      interface.sendState(STATE_OPEN_HORIZ,openHorizB[0],openHorizB[1],openHorizB[2],openHorizB[3],openHorizB[4],PAD);
      break;

  //THIS is the permanent open loop case which is currently everywhere
    case CMD_OPEN_VERT:
      Serial.println("CMD OPEN LOOP VERT");
      for(int i = 1; i<=5; i++){
        //turning off for sure case
        if(command.data[i]==0){
          openVertB[i-1]=false;
          analogWrite(actuatorPinZ[i-1],0);
          motorController.updateAZ(i,0);
        }
        //the going in case
        else if(command.data[i]<128){
          openVertB[i-1]=true;
          digitalWrite(valveSelectZ[i-1],LOW);
          analogWrite(actuatorPinZ[i-1],200);
          openVertT[i-1]=millis();
          motorController.updateAZ(i,200);
        }
        //the going out case
        else{
          openVertB[i-1]=true;
          digitalWrite(valveSelectZ[i-1],HIGH);
          analogWrite(actuatorPinZ[i-1],200);
          openVertT[i-1]=millis();
          motorController.updateAZ(i,200);
        }
      }
      interface.sendState(STATE_OPEN_VERT,openVertB[0],openVertB[1],openVertB[2],openVertB[3],openVertB[4],PAD);
      break;
      
      case CMD_OUTPUT:
        Serial.println("cmd output");
        Serial.print(PIDcontrollerX[0].getSensor());;Serial.print(" ");
        Serial.print(PIDcontrollerX[0].getSetpoint());;Serial.print(" ");
        Serial.print(PIDcontrollerX[0].getPID(0));;Serial.print(" ");
        Serial.print(PIDcontrollerX[0].updateOutput());Serial.print(" ");
        Serial.println();
        interface.sendState(STATE_OUTPUT,outputX[0],outputX[1],outputX[2],outputX[3],outputX[4],PAD);
        break;
      case CMD_GET_SETPOINT:
        Serial.println("cmd get setpoint");
        interface.sendState(STATE_GET_SETPOINT,PIDcontrollerX[0].getSetpoint(),PIDcontrollerX[1].getSetpoint(),PIDcontrollerX[2].getSetpoint(),PIDcontrollerX[3].getSetpoint(),PIDcontrollerX[4].getSetpoint(),PAD);
        break;
}//end of switch
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*+++++++++++++++++++++++++++++++++++++++output+++++++++++++++++++++++++++++++++++++++*/
/**********************************closed loop stuff***********************************/
  if(/*pidCalibrated == true &&*/ ClosedLoopMode == true){    
    //use PID to make actuator adjustments
    for(int i=1;i<5;i++){
      outputX[i]=PIDcontrollerX[i].updateOutput();
     // Serial.print(outputX[i],DEC);
    //  Serial.print(" ");
      motorController.updateAX(i,outputX[i]);
     // /*
//      PIDcontrollerZ[i].updateOutput();
    //  */
      if( ( (openVertB[i]==true) && ((millis()-openVertT[i]) > 500) ) || openVertB[i]== false){
        analogWrite(actuatorPinZ[i],0);
        openVertB[i]=false;
      }//end of the open loop timer check
/*this will come in later when Z has feedback      if(openVertB[i]==true){
        openVertB[i]=false;
      }*/
      if(openHorizB[i]==true){
        openHorizB[i]=false;
      }
    }
    motorController.updateMotor();
  }//end of calibrated PID output
/**********************************closed loop stuff**********************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$open loop minimal working$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
//In this operating mode, it's timing only
//master passes along the right information and does all the correct processing
//master has to be quite smart
  else if (OpenLoopMode == true){
    for(int i=0; i<5; i++){
     //these check and see how long the valve has been open for
     //if on long enough, close it right away
      if( ( (openVertB[i]==true) && (millis()-openVertT[i] > 500) ) || openVertB[i]==false){
        analogWrite(actuatorPinZ[i],0);
        openVertB[i]=false;
      }
      if( ( (openHorizB[i]==true) && (millis()-openHorizT[i] > 500) ) || openHorizB[i]==false){
        analogWrite(actuatorPinX[i],0);
        openHorizB[i]=false;
      }
    }
  }
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$open loop minimal working$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^no mode^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
//the perfect state for being off
  else{//not too sure what's safe to do here in the "don't move me" stage
    for(int i=0;i<5;i++){
      PIDcontrollerX[i].setSetPoint(PIDcontrollerX[i].getSensor());
      PIDcontrollerX[i].updateOutput();
     // /*
//      PIDcontrollerZ[i].setSetPoint(PIDcontrollerZ[i].getSensor());
//      PIDcontrollerZ[i].updateOutput();
    //  */
        if(openVertB[i]==true){
          analogWrite(actuatorPinZ[i],0);
          openVertB[i]=false;
        }
        if(openHorizB[i]==true){
          analogWrite(actuatorPinX[i],0);
          openHorizB[i]=false;
        }
      motorController.off();
      motorController.updateMotor();
    }
  }
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^no mode^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/  
/*+++++++++++++++++++++++++++++++++++++++output+++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//PERIODIC BATTERY CHECKING
//If battery is checked from time to time, message collission will have to be set in CAN Bus initialization and code handling in library files
  if((millis()-BcheckTIME)>50000){
    checkBat();
	BcheckTIME=millis();
  }
}

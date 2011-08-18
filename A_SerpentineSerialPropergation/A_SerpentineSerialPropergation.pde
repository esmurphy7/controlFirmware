/*
 potentiometer feedback based, left/right actuated
 send 0 or 1 to signify direction to Serial 3
 direction is put into the front angle and
 the current angles are propergated down the module
 the end angle propergated out to the next module
 through Serial 2
 
 created July 9, 2011 Julian Fong
 (begining of the 
 second day of Vacnover 125
 celebrations)
 modified July 10, 2011 Julian Fong
 (begining of the 
 third day of Vacnover 125
 celebrations)
 modified July 10, 2011 Julian Fong
 used at end of Vacnover 125
 celebrations
 
 modified August 13, 2011 Julian Fong
 to incoperate sensor feed back and PID
 */

#include "boashield_pins.h"
#include "PIDcontrol.h"

//pin values
const int HORZ_POS_SENSOR[] = {
  HORZ_POS_SENSOR_1,
  HORZ_POS_SENSOR_2,
  HORZ_POS_SENSOR_3,
  HORZ_POS_SENSOR_4,
  HORZ_POS_SENSOR_5
};
const int HORZ_ACTUATOR_CTRL[] = {
  HORZ_ACTUATOR_CTRL_1,
  HORZ_ACTUATOR_CTRL_2,
  HORZ_ACTUATOR_CTRL_3,
  HORZ_ACTUATOR_CTRL_4,
  HORZ_ACTUATOR_CTRL_5
};
const int HORZ_ACTUATOR[] = {
  HORZ_ACTUATOR_1,
  HORZ_ACTUATOR_2,
  HORZ_ACTUATOR_3,
  HORZ_ACTUATOR_4,
  HORZ_ACTUATOR_5
};

//direction of mux
boolean even = false;

PIDcontrol PIDcontroller[] = {
  PIDcontrol(HORZ_POS_SENSOR[0], HORZ_ACTUATOR_CTRL[0], HORZ_ACTUATOR[0], even),
  PIDcontrol(HORZ_POS_SENSOR[1], HORZ_ACTUATOR_CTRL[1], HORZ_ACTUATOR[1], !even),
  PIDcontrol(HORZ_POS_SENSOR[2], HORZ_ACTUATOR_CTRL[2], HORZ_ACTUATOR[2], even),
  PIDcontrol(HORZ_POS_SENSOR[3], HORZ_ACTUATOR_CTRL[3], HORZ_ACTUATOR[3], !even),
  PIDcontrol(HORZ_POS_SENSOR[4], HORZ_ACTUATOR_CTRL[4], HORZ_ACTUATOR[4], even),
};

//motorSpeed
const int MOTOR_SPEED = 90;

// current angles of joints
//0 & 1 relay angle 3:undefined for initlal start
char angleArray[] = {
  '3','3','3','3','3'};

//when to send out a ready signal
char meReady = false;
char tailReady = false;

//limits on sensor at end of range
int highRange[] = {
  1023,1023,1023,1023,1023};
int lowRange[] = {
  0,0,0,0,0};

//place in the chain of the snake
byte myModuleNumber;
byte endModuleNumber;

unsigned long startTime[] = {
  0,0,0,0,0};
const unsigned long MAX_WAIT_TIME = 2000;

#define HEAD_SERIAL Serial3
#define TAIL_SERIAL Serial2

void setup(){
  //turn of motor pin
  pinMode(MOTOR_CONTROL,OUTPUT);
  analogWrite(MOTOR_CONTROL,0);
  //turn off actuator valves
  for(int i=0;i<5;i++){
    pinMode(HORZ_ACTUATOR[i],OUTPUT);
    analogWrite(HORZ_ACTUATOR[i],0);
  }
  //initilize output pins
  for(int i=0;i<5;i++){
    pinMode(HORZ_ACTUATOR_CTRL[i],OUTPUT);
  }
  //inilize sensor
  for(int i=0;i<5;i++){
    pinMode(HORZ_POS_SENSOR[i],INPUT);
  }

  for(int i=0;i<5;i++){
    PIDcontroller[i].setConstants(25,0,0);
  }

  delay(500);

  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial.begin(115200);
}

void loop(){
  //unsigned long prevMessageTime = 0;
  char headAngle = '3';
  char tailAngle = '3';
  
  byte voltage = 255;

  if(HEAD_SERIAL.available()>0){
    switch(HEAD_SERIAL.read()){
    case 's':
      if(meReady == false){
        break;
      }
      //setpionts to come
      while(HEAD_SERIAL.available()<1){
        delay(1);
      }
      headAngle = HEAD_SERIAL.read();
      break;

    case 'v':
      TAIL_SERIAL.write('v');
      Serial.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,250));
      break;

    case 'c':
      //calabration
      while(HEAD_SERIAL.available()<2){
        delay(1);
      }

      //my module number and end module number to come
      //brain is 0, first moving segment is 1 etc
      myModuleNumber = HEAD_SERIAL.read();
      endModuleNumber = HEAD_SERIAL.read();

      even = (myModuleNumber%2)==0;

      TAIL_SERIAL.write('c');
      TAIL_SERIAL.write(myModuleNumber+1);
      TAIL_SERIAL.write(endModuleNumber);

      //calibrate and reset everything
      calibrate();
      HEAD_SERIAL.flush();
      TAIL_SERIAL.flush();

      delay(1000);

      break;
    }
    
    HEAD_SERIAL.flush();
  }

  //check if snake lowwer down on chain is ready
  if(TAIL_SERIAL.available()>0){
    switch(TAIL_SERIAL.read()){
    case 'r':
      tailReady = true;
      break;

    case 'v':
      while(TAIL_SERIAL.available()<1){
        delay(1);
      }
      voltage = TAIL_SERIAL.read();

      HEAD_SERIAL.write('v');
      if(voltage < (analogRead(BAT_LEVEL_24V)/8) && myModuleNumber == endModuleNumber){
        HEAD_SERIAL.write(voltage);
      }
      else{
        HEAD_SERIAL.write(analogRead(BAT_LEVEL_24V)/8);
      }
      break;
    }
    TAIL_SERIAL.flush();
  }

  //head angles not '0' or '1' are not recognized
  if(headAngle == '0' || headAngle != '1'){
    //read new angle and propergate it down
    tailAngle = propergate(headAngle);

    //send the next angle down
    TAIL_SERIAL.write('s');
    TAIL_SERIAL.write(tailAngle);
  }

  //move the snake into position
  move();

  //check if ready and send signal if it is
  ready();
}



void ready(){
  //return ready signal if end module and s ready
  if(myModuleNumber == endModuleNumber && meReady == true){
    HEAD_SERIAL.write('r');
  }
  //else wait till tail is ready self is ready
  else if(tailReady == true && meReady == true){
    HEAD_SERIAL.write('r');
    
    meReady = false;
    tailReady = false;
  }
}


int propergate(int headAngle){
  char endAngle;

  for(int i=4;i>0;i--){
    if(angleArray[i] != angleArray[i-1]){
      startTime[i] = millis();
    }
    angleArray[i] = angleArray[i-1];
  }

  //on the first module
  //make head opposite of first segment
  // so it looks like it is facing forward
  if(myModuleNumber == '1'){
    angleArray[1] = headAngle;
    if(headAngle == '0'){
      angleArray[0] = '1';
    }
    else if(headAngle == '1'){
      angleArray[0] = '0';
    }
    startTime[0] = startTime[1];
  }
  else{
    //new angle is a new angle at front and the rest propergate
    angleArray[0] = headAngle;
  }

  //return angle that gets propergated out
  return endAngle;
}


//when to start propergate
//distance out of 255 from end to be ready to propergate
const int PROPERGATE_ZONE = 50;
//range out of 255 to not move 
const int DEAD_ZONE = 5;

void move(){
  boolean moved = false;
  int goal;

  //turn off motor, will turn back on if need moving
  //time gap will be covered by low pass filter on motor control
  analogWrite(MOTOR_CONTROL, 0);

  moved = true;

  //!!DANGER!! added because ther is currently no position sensor in head
  if(myModuleNumber =='1' &&
    (millis()-startTime[0]) < MAX_WAIT_TIME){
    if(angleArray[0] == '0'){
      digitalWrite(HORZ_ACTUATOR_CTRL[0], LOW);
    }
    else if(angleArray[0] == '1'){
      digitalWrite(HORZ_ACTUATOR_CTRL[0], HIGH);
    }
    analogWrite(HORZ_ACTUATOR[0],255);
  }
  //!!DANGER END!!


  for(int i=0;i<5;i++){
    int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

    //where to move to for '0' and '1' in angle array
    if(angleArray[i]=='0'){
      PIDcontroller[i].setSetPoint(map(  10 ,0,255,lowRange[i],highRange[i]));
      goal = 10;
    }
    else if(angleArray[i]=='1'){
      PIDcontroller[i].setSetPoint(map( 245 ,0,255,lowRange[i],highRange[i]));
      goal = 245;
    }

    //check if keep moving, ready to propergate, or stop
    if(abs(currentAngle-goal)>DEAD_ZONE &&
      angleArray[i] != '3' && 
      (millis()-startTime[i]) < MAX_WAIT_TIME){
      analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
      PIDcontroller[i].updateOutput();
      if(abs(currentAngle-goal)>PROPERGATE_ZONE){
        moved=false;
      }
    }
    else{
      analogWrite(HORZ_ACTUATOR[i],0);
    }
  }

  if(moved == true){
    meReady = true;
  }

  //turn off motor
  analogWrite(MOTOR_CONTROL, 0);
}


void calibrate(){
  //calibrade low and high range on snake

    //move to side when ctrl is HIGH 
  analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
  for(int i=0;i<5;i++){
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(5000);
  //stop
  analogWrite(MOTOR_CONTROL,0);
  for(int i=0;i<5;i++){
    analogWrite(HORZ_ACTUATOR[i],0);
  }
  delay(1000);
  //set Range
  for(int i=0;i<5;i++){
    highRange[i] = analogRead(HORZ_POS_SENSOR[i]);
  }


  //move to side when ctrl is LOW
  analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
  for(int i=0;i<5;i++){
    digitalWrite(HORZ_ACTUATOR_CTRL[i], LOW);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(5000);
  //stop
  analogWrite(MOTOR_CONTROL,0);
  for(int i=0;i<5;i++){
    analogWrite(HORZ_ACTUATOR[i],0);
  }
  delay(1000);
  //set Range
  for(int i=0;i<5;i++){
    lowRange[i] = analogRead(HORZ_POS_SENSOR[i]);
  }

  //adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++){
    if(highRange[i]>lowRange[i]){
      PIDcontroller[i].setEven(true);
      angleArray[i] = '0';
    }
    else{
      int temp = highRange[i];
      highRange[i] = lowRange[i];
      lowRange[i] = temp;
      PIDcontroller[i].setEven(false);
      angleArray[i] = '1';
    }
  }

  //print to serial the ranges and even for diagnostics
  Serial.print("\n");
  for(int i=0;i<5;i++){
    Serial.print(highRange[i]);
    Serial.print(' ');
  }
  Serial.println("HIGH");

  for(int i=0;i<5;i++){
    Serial.print(lowRange[i]);
    Serial.print(' ');
  }
  Serial.println("LOW");

  for(int i=0;i<5;i++){
    Serial.print(angleArray[i]);
    Serial.print('   ');
  }
  Serial.println("position");
}














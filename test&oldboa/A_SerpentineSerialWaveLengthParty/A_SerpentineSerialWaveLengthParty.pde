/*
timing based, full left/right actuated
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

//NOTES MARKED "DANGER" INDICATES MODIFICATIONS TO CODE TO MOVE HEAD
//(the big snake head not first segment as I refor to in the code)
// DURING WAVELENGTH PARTY WHICH HAD NO SENSOR FEED BACK IN HEAD
// DO NOT PROPERGATE THIS CODE WITHOUT ADDJUSTMENT


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

//limits on sensor at end of range
int highRange[] = {
  1023,1023,1023,1023,1023};
int lowRange[] = {
  0,0,0,0,0};

//place in the chain of the snake
byte myModuleNumber;
byte endModuleNumber;

#define HEAD_SERIAL Serial
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
    PIDcontroller[i].setConstants(50,0,0);
  }

  delay(500);

  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial.begin(115200);
}

int left = 0;
int right = 0;
int phase = 0;
//ttue is right
boolean direction = true;
boolean automatic = false;

void loop(){
  //unsigned long prevMessageTime = 0;
  char headAngle = '3';
  char tailAngle = '3';

  if(HEAD_SERIAL.available()>0){
    switch(HEAD_SERIAL.read()){
    case 's':
      //setpionts to come
      while(HEAD_SERIAL.available()<1){
        delay(1);
      }
      headAngle = HEAD_SERIAL.read();
      break;

    case 'v':
      Serial.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,25000));
      break;
    case 'c':
      //calabration
      //tell next module to also calibrate
      TAIL_SERIAL.write('c');
      //my module number and end module number to come
      //brain is 0, first moving segment is 1 etc
      while(HEAD_SERIAL.available()<2){
        delay(1);
      }
      myModuleNumber = HEAD_SERIAL.read();
      endModuleNumber = HEAD_SERIAL.read();

      even = (myModuleNumber%2)==0;

      TAIL_SERIAL.write(myModuleNumber+1);
      TAIL_SERIAL.write(endModuleNumber);

      //calibrate and reset everything
      calibrate();
      HEAD_SERIAL.flush();
      TAIL_SERIAL.flush();

      delay(1000);
      ready();

      break;

    case 'u':
      left = 5;
      right = 5;
      automatic = true;
      break;
    case 'l':
      left = 5;
      right = 4;
      automatic = true;
      break;
    case 'r':
      left = 4;
      right = 5;
      automatic = true;
      break;
    case 'd':
      automatic = false;
      break;
    }
    
    //flush everything and get ready for next loop
    Serial2.flush();
    Serial3.flush();
    Serial.flush();
  }

  if(automatic == true){
    if(direction){
      headAngle = '1';
      phase++;
      if(phase>=right){
        phase = 0;
        direction=false;
      }
    }
    else{
      headAngle = '0';
      phase++;
      if(phase>=left){
        phase = 0;
        direction=true;
      }
    }
    //DANGER DANGER
    //automatic = false;
  }

  //head angles not '0' or '1' are not recognized
  if(headAngle != '0' && headAngle != '1'){
    return;
  }

  //read new angle and propergate it down
  tailAngle = propergate(headAngle);

  //send the next angle down
  TAIL_SERIAL.write(tailAngle);

  //move the snake into position
  move();

  //turn off all actuators and motors(backup)
  for(int i=0;i<5;i++){
    analogWrite(HORZ_ACTUATOR[i],0);
  }
  analogWrite(MOTOR_CONTROL, 0);

  //ready to go one
  ready();
  
  /*
  //flush everything and get ready for next loop
  Serial2.flush();
  Serial3.flush();
  Serial.flush();
  */
}

void ready(){
  //return ready signal if end module
  if(myModuleNumber == endModuleNumber){
    HEAD_SERIAL.write('r');
  }
  //else wait till ready signal and send own ready
  else{
    do{
      while(TAIL_SERIAL.available()<1){
        delay(1);
      }
    }
    while(TAIL_SERIAL.read() != 'r');

    HEAD_SERIAL.write('r');
  }
}


int propergate(int headAngle){
  char endAngle;

  //DANGER propergating to first and 
  //make head opposite of first segment
  // so it looks like it is facing forward

  for(int i=4;i>0;i--){
    angleArray[i] = angleArray[i-1];
  }
  angleArray[1] = headAngle;
  if(headAngle == '0'){
    angleArray[0] = '1';
  }
  else if(headAngle == '1'){
    angleArray[0] = '0';
  }

  //return angle that gets propergated out
  return endAngle;


  /*
  //new angle is a new angle at front and the rest propergate
   for(int i=4;i>0;i--){
   angleArray[i] = angleArray[i-1];
   }
   angleArray[0] = headAngle;
   
   //return angle that gets propergated out
   return endAngle;
   */
}


//time to try moving actuator before giving up
const int MAX_WAIT_TIME = 3000;
//when to start propergate
//distance out of 255 from end
const int PROPERGATE_ZONE = 40;
//safty range to not move 
const int DEAD_ZONE = 10;

//DANGER added for no fead back head
char prevHEADangle = '3';

void move(){
  //move till angleArray is matched
  boolean moved = false;

  int goal;

  for(int i=0;i<5;i++){
    if(angleArray[i]=='0'){
      PIDcontroller[i].setSetPoint(map(DEAD_ZONE,0,255,lowRange[i],highRange[i]));
    }
      else if(angleArray[i]=='1'){
      PIDcontroller[i].setSetPoint(map(255-DEAD_ZONE,0,255,lowRange[i],highRange[i]));
    }
  }

  //added for no head control

  //DANGER move head
  if(angleArray[0] != prevHEADangle){
    if(angleArray[0] == '0'){
      digitalWrite(HORZ_ACTUATOR_CTRL[0], LOW);
    }
    else if(angleArray[0] == '1'){
      digitalWrite(HORZ_ACTUATOR_CTRL[0], HIGH);
    }
    analogWrite(HORZ_ACTUATOR[0],255);
    analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
    prevHEADangle = angleArray[0];
    delay(100);
  }

  //time limit so if stuck, goes to next
  unsigned long startTime = millis();
  while(moved == false && (millis()-startTime) < MAX_WAIT_TIME){
    moved = true;

    //DANGER i adjusted to =1 for head with no sensor feed back
    for(int i=1;i<5;i++){
      int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

      if(angleArray[i]=='0'){
        goal = 10;
      }
      else if(angleArray[i]=='1'){
        goal = 245;
      }

      if(abs(currentAngle-goal)>DEAD_ZONE){
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
  }

  //turn of motor
  analogWrite(MOTOR_CONTROL, 0);
  //turn off all actuators
  for(int i=0;i<5;i++){
    analogWrite(HORZ_ACTUATOR[i],0);
  }
}


void calibrate(){
  //calibrade low and high range on snake

    //move to side when ctrl is HIGH 
  analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
  for(int i=0;i<5;i++){
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
    //delay(2000);
    //analogWrite(HORZ_ACTUATOR[i],0);
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
    //delay(2000);
    //analogWrite(HORZ_ACTUATOR[i],0);
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

  /*
  //printing out calibration
   for(int i=0;i<5;i++){
   Serial.print(highRange[i]);
   Serial.print(' ');
   }
   Serial.println("HIGH ");
   for(int i=0;i<5;i++){
   Serial.print(lowRange[i]);
   Serial.print(' ');
   }
   Serial.println("LOW ");
   */
}






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
 modified August 29
 made at burning man
 does not wait for ready, angles move as
 fast as they  are pushed in
 -digging into sand, need vertical control
 
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

const int VERT_ACTUATOR_CTRL[] = {
  VERT_ACTUATOR_CTRL_1,
  VERT_ACTUATOR_CTRL_2,
  VERT_ACTUATOR_CTRL_3,
  VERT_ACTUATOR_CTRL_4,
  VERT_ACTUATOR_CTRL_5
};
const int VERT_ACTUATOR[] = {
  VERT_ACTUATOR_1,
  VERT_ACTUATOR_2,
  VERT_ACTUATOR_3,
  VERT_ACTUATOR_4,
  VERT_ACTUATOR_5
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
const int MOTOR_SPEED = 130;

// current angles of joints
//0 & 1 relay angle 2:strundefined for initlal start
char horzAngleArray[] = {
  '3','3','3','3','3'};
char vertAngleArray[] = {
  '3','3','3','3','3'};

unsigned long horzTimerArray[] = {
  0,0,0,0,0};

unsigned long vertTimerArray[] = {
  0,0,0,0,0};

//limits on sensor at end of range
int highRange[] = {
  1023,1023,1023,1023,1023};
int lowRange[] = {
  0,0,0,0,0};

//place in the chain of the snake
char myModuleNumber;
char endModuleNumber;

//upstearm == 1
//down == 2
#define HEAD_SERIAL Serial1
#define TAIL_SERIAL Serial2

void setup(){
  //turn of motor pin
  pinMode(MOTOR_CONTROL,OUTPUT);
  analogWrite(MOTOR_CONTROL,0);
  //turn off actuator valves
  for(int i=0;i<5;i++){
    pinMode(HORZ_ACTUATOR[i],OUTPUT);
    analogWrite(HORZ_ACTUATOR[i],0);
    pinMode(VERT_ACTUATOR[i],OUTPUT);
    analogWrite(VERT_ACTUATOR[i],0);
  }
  //initilize output pins
  for(int i=0;i<5;i++){
    pinMode(HORZ_ACTUATOR_CTRL[i],OUTPUT);
    pinMode(VERT_ACTUATOR_CTRL[i],OUTPUT);
  }
  //inilize sensor
  for(int i=0;i<5;i++){
    pinMode(HORZ_POS_SENSOR[i],INPUT);
  }

  for(int i=0;i<5;i++){
    PIDcontroller[i].setConstants(50,0,0);
  }

  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial1.begin(115200);
  Serial.begin(115200);
  
  
  delay(1000);
}

char headAngle[] = {
  '3','3'  };
char tailAngle[] = {
  '3','3'  };

void loop(){
  headAngle[0] = '3';
  headAngle[1] = '3';

  char message;

  if(HEAD_SERIAL.available()>0){
    //*
    Serial.print("\nrecieved ");
    Serial.write(HEAD_SERIAL.peek());
    //*/
    switch(HEAD_SERIAL.read()){
    case 's':
      //setpionts to come
      while(HEAD_SERIAL.available()<2){
        delay(1);
      }
      headAngle[0] = HEAD_SERIAL.read();
      headAngle[1] = HEAD_SERIAL.read();

      break;

    case 'v':
      Serial.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,25000));
      break;
    case 'c':
      //calabration
      //my module number and end module number to come
      //brain is 0, first moving segment is 1 etc
      while(HEAD_SERIAL.available()<2){
        delay(1);
      }

      //tell next module to also calibrate
      TAIL_SERIAL.write('c');
      myModuleNumber = HEAD_SERIAL.read();
      endModuleNumber = HEAD_SERIAL.read();

      even = (myModuleNumber%2)==0;

      TAIL_SERIAL.write(myModuleNumber+1);
      TAIL_SERIAL.write(endModuleNumber);

      //calibrate and reset everything
      calibrate();
      HEAD_SERIAL.flush();
      TAIL_SERIAL.flush();

      delay(100);
      ready();
      break;

    case 'h':
      //for moving head, turns on and off motor
      while(HEAD_SERIAL.available()<1){
        delay(1);
      }

      message = HEAD_SERIAL.read();
      if(message == '5'){
        digitalWrite(HORZ_ACTUATOR_CTRL[0],LOW);
      }
      else if(message == '6'){
        digitalWrite(HORZ_ACTUATOR_CTRL[0],HIGH);
      }
      
      if(message == '5' || message == '6'){
        for(int i=0;i<=255;i++){
          analogWrite(HORZ_ACTUATOR[0],i);
          delay(2);
        }
        for(int i=255;i>=0;i--){
          analogWrite(HORZ_ACTUATOR[0],i);
          delay(2);
        }
      }

      analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
      delay(1000);
      analogWrite(MOTOR_CONTROL,0);
      break;

    case 'l':
      TAIL_SERIAL.print("l");
      strighten();
      ready();
      break;
    }

    //flush everything and get ready for next loop
    HEAD_SERIAL.flush();
  }

  if(TAIL_SERIAL.available()>0){
    switch(TAIL_SERIAL.read()){
    case 'r':
      //setpionts to come
      while(TAIL_SERIAL.available()<1){
        delay(1);
      }
      HEAD_SERIAL.write('r');
      HEAD_SERIAL.write(TAIL_SERIAL.read());

      break;
    }
    TAIL_SERIAL.flush();
  }


  //move the snake into position
  move();

  //head angles not '0', '1' or '2' are not recognized
  if(headAngle[0] != '0' &&
    headAngle[0] != '1' &&
    headAngle[0] != '2' &&
    headAngle[1] != '0' &&
    headAngle[1] != '1' &&
    headAngle[1] != '2'){
    return;
  }

  //read new angle and propergate it down
  propergate();

  TAIL_SERIAL.write('s');
  //send the next angle down
  TAIL_SERIAL.write(tailAngle[0]);
  TAIL_SERIAL.write(tailAngle[1]);


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
  HEAD_SERIAL.write('r');
  HEAD_SERIAL.write(myModuleNumber);
}


void propergate(){
  tailAngle[0] = horzAngleArray[4];
  tailAngle[1] = vertAngleArray[4];

  for(int i=4;i>0;i--){
    if(horzAngleArray[i] != horzAngleArray[i-1]){
      horzTimerArray[i] = millis();
    }
    horzAngleArray[i] = horzAngleArray[i-1];

    if(vertAngleArray[i] != vertAngleArray[i-1]){
      vertTimerArray[i] = millis();
    }
    vertAngleArray[i] = vertAngleArray[i-1];
  }

  if(horzAngleArray[0] != headAngle[0]){
    horzTimerArray[0] = millis();
  }
  horzAngleArray[0] = headAngle[0];

  if(vertAngleArray[0] != headAngle[1]){
    vertTimerArray[0] = millis();
  }
  vertAngleArray[0] = headAngle[1];

  //*
  Serial.write('\n');
  for(int i=0;i<5;i++){
    Serial.print(horzAngleArray[i]);
  }
  Serial.println(' horz');

  Serial.write('\n');
  for(int i=0;i<5;i++){
    Serial.print(vertAngleArray[i]);
  }
  Serial.println(' vert');
  //*/
}


//time to try moving actuator before giving up
const int MAX_WAIT_TIME = 3000;
//safty range to not move 
const int DEAD_ZONE = 10;

int count=0;

void move(){
  //to see if any in segment moved
  //so we can turn of motor when it's reached setpoint
  boolean moved = false;

  int goal;

  for(int i=0;i<5;i++){
    //horizontal
    if(horzAngleArray[i]=='0'){
      goal = 10;
    }
    else if(horzAngleArray[i]=='1'){
      goal = 245;
    }
    else if(horzAngleArray[i]=='2'){
      goal = 127;
    }
    else{
      analogWrite(HORZ_ACTUATOR[i],0);
      continue;
    }

    PIDcontroller[i].setSetPoint(map(goal,0,255,lowRange[i],highRange[i]));

    int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

    if((abs(currentAngle-goal)>DEAD_ZONE) &&
      (millis()-horzTimerArray[i]) < MAX_WAIT_TIME){
      analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
      PIDcontroller[i].updateOutput();
      moved = true;
    }
    else{
      analogWrite(HORZ_ACTUATOR[i],0);
    }

    //vertical
    if((millis()-vertTimerArray[i]) < MAX_WAIT_TIME){
      analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
      if(vertAngleArray[i] == '0'){
        digitalWrite(VERT_ACTUATOR_CTRL[i],LOW);
        analogWrite(VERT_ACTUATOR[i],255);
        moved = true;
      }
      else if(vertAngleArray[i] == '1'){
        digitalWrite(VERT_ACTUATOR_CTRL[i],HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
        moved = true;
      }
      else if(vertAngleArray[i] == '2'){
        //do nothing
      }
    }
    else{
      analogWrite(VERT_ACTUATOR[i],0);
    }

  }
  if(moved == false){
    analogWrite(MOTOR_CONTROL, 0);
  }
}


void calibrate(){
  //calibrade low and high range on snake

    //move to side when ctrl is HIGH 
  analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
  for(int i=0;i<5;i++){
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(3000);
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
  delay(3000);
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
      horzAngleArray[i] = '1';
    }
    else{
      int temp = highRange[i];
      highRange[i] = lowRange[i];
      lowRange[i] = temp;
      PIDcontroller[i].setEven(false);
      horzAngleArray[i] = '0';
    }
  }

  strighten();

  //*
  //printing out calibration
  Serial.println("");
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
  for(int i=0;i<5;i++){
    Serial.print(PIDcontroller[i].getEven(),BIN);
    Serial.print("    ");
  }
  Serial.println("EVEN");
  //*/
}

//uses the same constants used by move()
void strighten(){
  //move till horzAngleArray is matched
  boolean inPosition = false;

  int goal = 127;
  for(int i=0;i<5;i++){
    PIDcontroller[i].setSetPoint(map(goal,0,255,lowRange[i],highRange[i]));
  }

  //time limit so if stuck, goes to next
  unsigned long startTime = millis();
  while(inPosition == false && (millis()-startTime) < MAX_WAIT_TIME){
    inPosition = true;

    for(int i=0;i<5;i++){
      int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

      if(abs(currentAngle-goal)>DEAD_ZONE){
        analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
        PIDcontroller[i].updateOutput();
        inPosition=false;
      }
      else{
        analogWrite(HORZ_ACTUATOR[i],0);
      }
    }
  }

  //turn of motor
  analogWrite(MOTOR_CONTROL, 0);
  //turn off all actuators
  //undefine angles
  for(int i=0;i<5;i++){
    analogWrite(HORZ_ACTUATOR[i],0);
    horzAngleArray[i] = '2';
  }
}








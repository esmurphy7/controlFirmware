#include "PIDcontrol.h" //Julian PID header
#include "motorControl.h"

/*X is for horizontal
  Z is for vertical*/
//155 - 107 is our approximate POT range


//analog pins for pot reading
const int sensorPinX = A0;

//PID signal pins
const int actuatorPinX = 11;//and 12 for down

//Valve select pins because we don't have enough PWM
const int valveSelectX = 36;

//Dither will be fed into all unselected valves
//const int ditherPin = 13;

//motor write speed
const int motorPin = 2;

motorControl	motorController(motorPin);

/*limit switches in series. will determine L/R through math*/
const int limit_switchX = 26;

//PID controllers
PIDcontrol PIDcontrollerX( sensorPinX, actuatorPinX, limit_switchX, valveSelectX);

//motor
int motorValue = 130;

byte cur_pid;
int PERIOD=1000;

boolean sineWave = false;

void setup() {
  Serial.begin(115200);
  
  for(int i=0;i<=53;i++){
    pinMode(i, INPUT);
  }
  
  pinMode(motorPin,OUTPUT);
  digitalWrite(motorPin,0);
  
  pinMode(sensorPinX,INPUT);
  pinMode(actuatorPinX,OUTPUT);
  pinMode(valveSelectX,OUTPUT);
  
  Serial.println("press 'p' to continue");
  boolean gogo = false;
  while(gogo==false){
    if(Serial.read() == 'p'){
      gogo = true;
    }
  }
  PIDcontrollerX.setConstants(3, 0, 1);
  delay(1000);
  Serial.println("main program");
  
  cur_pid = PIDcontrollerX.getSensor();
  PIDcontrollerX.setSetPoint(cur_pid);
}

int outputX = 0;

void loop() {  
  
  if(Serial.available()){
    byte tmp = Serial.read();
    Serial.println(tmp);
    if(tmp=='a') {cur_pid+=10;}
    if(tmp=='z') {cur_pid-=10;}

    if(tmp=='q') {
      Serial.println(motorController.getSpeed()); 
    }
    if(tmp=='w'){
      Serial.println(motorController.getMax());
    }
    if(tmp=='e'){
      cur_pid = PIDcontrollerX.getSensor();
      PIDcontrollerX.setSetPoint(cur_pid);
      outputX=PIDcontrollerX.updateOutput();
      motorController.updateAX(0,outputX);
      motorController.updateMotor();
      Serial.println(motorController.getSpeed());
    }
    if(tmp=='r'){
        Serial.print(PIDcontrollerX.getSensor());
      
      Serial.println(" ");
    }
    if(tmp=='y'){
      Serial.print("200 speed ");
      digitalWrite(motorPin,200);
      delay(1000);
      Serial.print("100 speed ");
      digitalWrite(motorPin,100);
      delay(1000);
      Serial.print("0 speed ");
      digitalWrite(motorPin,0);
      delay(1000);
      Serial.println("motor test done");
    }
    
    if(tmp=='u'){
        Serial.print(PIDcontrollerX.getSetpoint());
        Serial.print(" ");
      
      Serial.println();
    }
    
    if(tmp == 'i'){
      char tmp2[3];
      for(int i=0;i<3;i++){
        tmp2[i]=Serial.read();
      }
      int tmp3=atoi(tmp2);
      Serial.print("atoi output");
      Serial.print(" ");
      Serial.print(tmp3);
      Serial.println(" ");
      analogWrite(motorPin,tmp3);
    }
    if(tmp=='o') {
      Serial.println(outputX);
    }
    if(tmp=='Q'){
      analogWrite(3,200);
      analogWrite(4,0);
      analogWrite(motorPin,150);
      delay(1000);
      analogWrite(4,200);
      analogWrite(3,0);
      delay(1000);
      analogWrite(motorPin,0);
    }
    if(tmp == 's'){
      sineWave = !sineWave;
    }
    if(tmp == 'd'){
      PERIOD+=100;
    }
    if(tmp=='c'){
      PERIOD-=100;
    }
  }//end of tmp read

  if(sineWave){
    cur_pid = 10*sin((millis()*2*PI)/PERIOD) + 130;
  }
  
  PIDcontrollerX.setSetPoint(cur_pid);

  outputX=PIDcontrollerX.updateOutput();
  motorController.updateAX(0,abs(outputX));

  motorController.updateMotor();
}

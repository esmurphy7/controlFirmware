#include "PIDcontrolBOA.h"
#include "motorControl.h"
#include "boashield_pins.h"

/*X is for horizontal
  Z is for vertical*/

//analog pins for pot reading
const int sensorPinX = HORZ_POS_SENSOR_1;

//PID signal pins
const int actuatorPinX = HORZ_ACTUATOR_1;

//Valve select pins because we don't have enough PWM
const int valveSelectX = HORZ_ACTUATOR_CTRL_1;

//Dither will be fed into all unselected valves
const int ditherPin = DITHER;

//motor write speed
const int motorPin = MOTOR_CONTROL;

motorControl	motorController(motorPin);

/*limit switches in series. will determine L/R through math*/
const int limit_switchX = HORZ_LIMIT_SWITCH_1;

//PID controllers
PIDcontrol PIDcontrollerX( sensorPinX, actuatorPinX, limit_switchX, valveSelectX);

//motor
int motorValue = 150;

byte cur_pid;

int PERIOD=1000;
boolean sineWave = false;

int K_P = 5;
int K_I = 10;
int K_D = 1;

//155 - 107 WAS our approximate POT range for sensor test 1
int Ain = 0;
int Aout = 255;

int ditherFreq = 200;

void setup() {
  
  for(int i = 2; i <= 13; i++){
    pinMode(i,OUTPUT);
    digitalWrite(i,0);
  }
  
  Serial.begin(115200);
  
  pinMode(motorPin,OUTPUT);
  digitalWrite(motorPin,0);
  
  pinMode(sensorPinX,INPUT);
  pinMode(actuatorPinX,OUTPUT);
  pinMode(valveSelectX,OUTPUT);
  
  Serial.println("press 'p' to calibrate");
  boolean gogo = false;
  while(gogo==false){
    if(Serial.read() == 'p'){
      gogo = true;
    }
  }
  calibrate();
  PIDcontrollerX.setConstants(K_P, K_I, K_D);
  delay(1000);
  Serial.println("main program");
  motorController.maxSet(motorValue);
  //cur_pid = PIDcontrollerX.getSensor();
  Serial.print("Moving to ");
  Serial.println(cur_pid,DEC);
  PIDcontrollerX.setSetPoint((Ain + Aout)/2);
}

int outputX = 0;

boolean motorTest = false;

void updateSystem() {
  PIDcontrollerX.setSetPoint(cur_pid);

  outputX=PIDcontrollerX.updateOutput();
  motorController.updateAX(0,abs(outputX));
  ditherF();
  motorController.updateMotor();
}

void bottom() {
  cur_pid = Ain;
  do {
    updateSystem();
  } while(PIDcontrollerX.getSensor() != Ain);
}

void top() {
  cur_pid = Aout;
  do {
    updateSystem();
  } while(PIDcontrollerX.getSensor() != Aout);
}

void calibrate() {
  /*PIDcontrollerX.setConstants(10, 0, 0);
  cur_pid = 0;
  updateSystem();
  delay(1000);
  Ain = PIDcontrollerX.getSensor();
  Serial.print("Low value is ");
  Serial.println(Ain,DEC);
  
  cur_pid = 255;
  updateSystem();
  delay(1000);
  Aout = PIDcontrollerX.getSensor();
  Serial.print("High value is ");
  Serial.println(Aout,DEC);
  PIDcontrollerX.setConstants(K_P, K_I, K_D);
  cur_pid = Aout - (Aout-Ain)/5;
  */
  //why you no work
  analogWrite(motorPin,130);
  analogWrite(actuatorPinX,0);
  digitalWrite(valveSelectX,HIGH);
  analogWrite(actuatorPinX,255);
  delay(2100);
  analogWrite(actuatorPinX,0);
  Aout = PIDcontrollerX.getSensor();
  Serial.print("High value is ");
  Serial.println(Aout,DEC);
  
  digitalWrite(valveSelectX,LOW);
  analogWrite(actuatorPinX,250);
  delay(2100);
  analogWrite(actuatorPinX,0);
  Ain = PIDcontrollerX.getSensor();
  analogWrite(motorPin,0);
  Serial.print("Low value is ");
  Serial.println(Ain,DEC);

  PIDcontrollerX.calibrated(Ain,Aout);

  cur_pid = Aout - (Aout-Ain)/5;
}

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
int ticks = 0;
int ticksPsecond = 0;
long lastmillis = 0;

void loop() {  
  if(Serial.available()){
    byte tmp = Serial.read();
    if(tmp == 'f') {
      Serial.print("Ticks per Second: ");
      Serial.println(ticksPsecond);
    }
    if(tmp == 'C') {
      calibrate();
    }
    if(tmp=='a') {cur_pid+=10;}
    if(tmp=='z') {cur_pid-=10;}
    if(tmp=='w'){
      Serial.println(motorController.getMax());
    }
    if(tmp=='e'){
      cur_pid = PIDcontrollerX.getSensor();
      Serial.print("Moving to ");
      Serial.print(cur_pid,DEC);
      Serial.print(" With motor speed ");
      PIDcontrollerX.setSetPoint(cur_pid);
      outputX=PIDcontrollerX.updateOutput();
      motorController.updateAX(0,outputX);
      motorController.updateMotor();
      Serial.println(motorController.getSpeed());
    }
    if(tmp=='u'){
      Serial.print("Sensor: ");
      Serial.println(PIDcontrollerX.getSensor(),DEC);
      Serial.print("Setpoint: ");
      Serial.println(PIDcontrollerX.getSetpoint(),DEC);
    }
    
    /*if(tmp == 'i'){
      //if(Serial.available() < 3) {
      ///  Serial.println("Needs 3 digits");
     // }
      //else {
        char tmp2[3];
        for(int i=0;i<3;i++){
          tmp2[i]=Serial.read();
        }
        int tmp3=atoi(tmp2);
        Serial.print("Motor set to ");
        Serial.print(" ");
        Serial.print(tmp3);
        Serial.println(" ");
        analogWrite(motorPin,tmp3);
        delay(1000);
        Serial.println("Done");
      //}
    }*/
    if(tmp=='o') {
      Serial.print("Output: ");
      Serial.println(outputX);
      Serial.print("Motor Speed: ");
      Serial.println(motorController.getSpeed()); 
    }
   /* if(tmp=='Q'){
      analogWrite(3,200);
      analogWrite(4,0);
      analogWrite(motorPin,150);
      delay(1000);
      analogWrite(4,200);
      analogWrite(3,0);
      delay(1000);
      analogWrite(motorPin,0);
    }*/
    if(tmp == 's'){
      sineWave = !sineWave;
    }
    if(tmp == 'd'){
      PERIOD+=100;
      Serial.print("Sine Period: ");
      Serial.println(PERIOD);
    }
    if(tmp=='c'){
      PERIOD-=100;
      Serial.print("Sine Period: ");
      Serial.println(PERIOD);
    }
    //testing pid smootheness for single actuator
/*    if(tmp=='f'){
//******************************************************      
//may need to be rearranged
//ask for set point # then go to Ain, then test
      cur_pid = PIDcontrollerX.getSensor();
      PIDcontrollerX.setSetPoint(cur_pid);
      motorController.updateAX(0,0);
      motorController.updateMotor();
      //should be off for sure now
      cur_pid = Ain;
      PIDcontrollerX.setSetPoint(cur_pid);
      int cur_sensor = PIDcontrollerX.getSensor();
      
      
     //choosing a valid motor value for the test
      motorTest = false;
      int testMi;
      Serial.println("choose motor value");
      while(motorTest == false){
        if(Serial.available() > 2){
          char testM[3];
          for(int i=0;i<3;i++){
            testM[i]=Serial.read();
          }
          testMi=atoi(testM);
          if((testMi>0) && (testMi<=255)){
            Serial.print("atoi Motor output");
            Serial.print(" ");
            Serial.print(testMi);
            Serial.println(" ");
            analogWrite(motorPin,testMi);
            motorTest = true;
          }//if valid number
          else{
            Serial.println("bad number");
          }
        }//if statement for valid number
      }//end of getting motor value
      
      while(abs(cur_pid - cur_sensor) > 5){
        PIDcontrollerX.updateOutput();
        cur_sensor = PIDcontrollerX.getSensor();
      }//move to start point
      digitalWrite(motorPin,0);
      //ask for number of points
      Serial.println("how many target points?");
      boolean pointSet = false;
      int numPointI;
      while(pointSet == false){
        if(Serial.available()>0){
          char numPointC = Serial.read();
          numPointI=atoi(&numPointC);
          Serial.print("number of points = ");
          Serial.println(numPointI);
          pointSet = true;
        }
      }
      int PointArray[numPointI+1];
      PointArray[0]=Ain;
      for(int i=1;i<(numPointI-1);i++){
        PointArray[i]=Ain + i*(abs(Ain-Aout)/numPointI);
      }
      PointArray[numPointI]=Aout;
      PIDcontrollerX.setSetPoint(Ain);
      PIDcontrollerX.updateOutput();
      int timeStart = millis();
      int timeStop;
      int SPreached;
      analogWrite(motorPin,testMi);
   //   for(int i=;i<=numPointI;i++){
        //going to leave now. START here
        //while( not really close to position aka deadzone constantly update pid output
      //go to position for that part of array
      //record time difference
      //output time difference and corresponding pid jump ie 1-2, 2-3  5-4 so on
      //}
    }//end of tmp f*/
    if(tmp == 'T') {
      bottom();
      long timeStart = millis();
      top();
      Serial.print("Up took ");
      Serial.println(millis() - timeStart);
      
      timeStart = millis();
      bottom();
      Serial.print("Down took ");
      Serial.println(millis() - timeStart);
      cur_pid = (Aout + Ain)/2;
    }
    if(tmp == 'B') {
      bottom();
    }
    if(tmp == 'G') {
      top();
    }
    if(tmp == 'P') {
      if(!Serial.available()) {
        Serial.println("Need Argument");
      }
      else {
        byte increment = Serial.read();
        if(increment == 'u' && K_P < 40) {
          K_P ++;
        }
        else if (increment == 'd' && K_P > 0){
          K_P --;
        }
        Serial.print("PID is ");
        Serial.print(K_P,DEC);
        Serial.print(" ");
        Serial.print(K_I,DEC);
        Serial.print(" ");
        Serial.print(K_D,DEC);
        Serial.println();
        PIDcontrollerX.setConstants(K_P, K_I, K_D);
      }
    }
    if(tmp == 'I') {
      if(!Serial.available()) {
        Serial.println("Need Argument");
      }
      else {
        byte increment = Serial.read();
        if(increment == 'u' && K_I < 40) {
          K_I ++;
        }
        else if (increment == 'd' && K_I > 0){
          K_I --;
        }
        Serial.print("PID is ");
        Serial.print(K_P,DEC);
        Serial.print(" ");
        Serial.print(K_I,DEC);
        Serial.print(" ");
        Serial.print(K_D,DEC);
        Serial.println();
        PIDcontrollerX.setConstants(K_P, K_I, K_D);
      }
    }
    if(tmp == 'D') {
      if(!Serial.available()) {
        Serial.println("Need Argument");
      }
      else {
        byte increment = Serial.read();
        if(increment == 'u' && K_D < 40) {
          K_D ++;
        }
        else if (increment == 'd' && K_D > 0) {
          K_D --;
        }
        Serial.print("PID is ");
        Serial.print(K_P,DEC);
        Serial.print(" ");
        Serial.print(K_I,DEC);
        Serial.print(" ");
        Serial.print(K_D,DEC);
        Serial.println();
        PIDcontrollerX.setConstants(K_P, K_I, K_D);
      }
    }
    if(tmp == 'V'){
      int batIN = analogRead(BAT_LEVEL_24V);
      int batREAL = map(batIN,0,1023,0,25000);
      Serial.print(batREAL);
      Serial.println(" mv current battery voltage");
    }
  }//end of tmp read

  if(sineWave){
    cur_pid = (Aout-Ain)/2*sin((millis()*2*PI)/PERIOD) + (Ain + Aout)/2;
  }
  updateSystem();
  ticks ++;
  if(ticks == 200) {
    ticksPsecond = 200000 / (millis()-lastmillis);
    ticks = 0;
    lastmillis = millis();
  }

}

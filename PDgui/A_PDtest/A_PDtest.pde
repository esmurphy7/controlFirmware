/*********************************************** 
PIDtest
tries to reach a set point given through serial Comunications
all messages recieved are 8 char followed by '!'

STOP#### - Stop
STAT#### - Start

SPOS#### - set position
GPOS#### - get Position
GDRV#### - get derivative
GPDV#### - get PD value

SSPD#### - set speed limitto
SACC#### - set acceleration limit
SCKP#### - set constant Kp
SCKD#### - set constant Kd

created:  Julian Fong  28 May 2011
modified: Julian Fong  09 Jun 2011
************************************************/

//define inout and output pins
const int outputPin0L = 9;
const int outputPin0R = 8;
const int sensorPin0  = 2;

//sofware limits for control
const int maxLimit = 1023;
const int minLimit = 0;


//variables
String message = "";
int setPoint = 100;
int sensorReading  = 0;
int difference = 0;
int derivative = 0;
int prevSensorReading = 0;
int prevTime = 0;
int PD = 0;
int prevPD = 0;
int Kp = 128;
int Kd = 0;
int speedLimit = 255;
int accelerationLimit = 255;
boolean stop = true;

//dummy variable
int i;

void setup(){
  Serial.begin(115200);
  analogWrite(outputPin0L, 0);
  analogWrite(outputPin0R, 0);
  delay(100);
  setPoint = analogRead(sensorPin0);
}

void loop(){
  /********************
   Recieving Messages
  ********************/
  if(Serial.available() >= 9){
    //get message
    message = "";
    for(i=0;i<8;i++){
      message = message+ (char)Serial.read();
    }
    if(Serial.read() != '!'){
      //serial out of sync, get it back in sync
      while(true){
        if(Serial.available()){
          if(Serial.read() == '!'){
            return;
          }
        }
      }
      return;
    }
    //cases for each message
    if(message.startsWith("STOP")){
      stop = true;
    }
    else if(message.startsWith("STAT")){
      stop = false;
    }
    
    if(message.startsWith("SPOS")){
      setPoint = (int)(message.charAt(4)-'0')*1000
                + (int)(message.charAt(5)-'0')*100
                + (int)(message.charAt(6)-'0')*10
                + (int)(message.charAt(7)-'0');
    }
    else if(message.startsWith("GPOS")){
      Serial.print("SPOS");
      Serial.print(printNumber(sensorReading));
      Serial.print('!');
    }
    else if(message.startsWith("GDRV")){
      Serial.print("SDRV");
      Serial.print(printNumber(derivative));
      Serial.print('!');
    }
    else if(message.startsWith("GPDV")){
      Serial.print("SPDV");
      Serial.print(printNumber(PD));
      Serial.print('!');
    }
    else if(message.startsWith("SSPD")){
      speedLimit = getNumber(message.substring(4));
      if(speedLimit>255){
        speedLimit = 255;
      }
      else if(speedLimit < 0){
        speedLimit = 0;
      }
    }
    else if(message.startsWith("SACC")){
      accelerationLimit = getNumber(message.substring(4));
      if(accelerationLimit>255){
        accelerationLimit = 255;
      }
      else if(accelerationLimit < 0){
        accelerationLimit = 0;
      }
    }
    else if(message.startsWith("SCKP")){
      Kp = getNumber(message.substring(4));
    }
    else if(message.startsWith("SCKD")){
      Kd = getNumber(message.substring(4));
    }
  }
  
  /******
  Stop
  ******/
  if(stop == true){
    analogWrite(outputPin0L, 0);
    analogWrite(outputPin0R, 0);
    delay(100);
    return;
  }
  
  /*************
   PD Controls
  *************/
  //map sensor readings to 0 to 200
  sensorReading = analogRead(sensorPin0);
  
  //check sofware limits
  if(sensorReading > maxLimit || sensorReading < minLimit){
    stop = true;
  }
  
  difference = sensorReading - setPoint;
  derivative = (sensorReading - prevSensorReading) / (millis() - prevTime);
  
  if(prevSensorReading != sensorReading){
    prevSensorReading = sensorReading;
    prevTime = millis();
  }
  
  PD = Kp * difference - Kd * derivative;
  PD = constrain(PD, -speedLimit, speedLimit);
  //add dithering
  PD = PD + 20*int(sin(float(millis()/2)));
  PD = constrain(PD, -255, 255);
  
  //acceleration limiter
  /*
  if(PD > prevPD + accelerationLimit){
    PD = prevPD + accelerationLimit;
  }
  else if(PD < prevPD - accelerationLimit){
    PD = prevPD - accelerationLimit;
  }
  prevPD = PD;
  */
  
  //set to valve outputs
  if(PD >= 0){
    analogWrite(outputPin0L, 0);
    analogWrite(outputPin0R, PD);
  }
  else{
    analogWrite(outputPin0R, 0);
    analogWrite(outputPin0L, -PD);
  }
  
}

//return string of a 4 digit number
String printNumber(int number){
  
  number = constrain(number, -1023, 1023);
  
  if(number < 0){
    number = -1 * number;
    number += 1023;
  }
  
  String string;
  string.concat((int)(number%10000)/1000);
  string.concat((int)(number%1000)/100);
  string.concat((int)(number%100)/10);
  string.concat((int)(number%10));
  return string;
}

//return a number
int getNumber(String string){
  return ((int)(string.charAt(0)-'0')*1000
          + (int)(string.charAt(1)-'0')*100
          + (int)(string.charAt(2)-'0')*10
          + (int)(string.charAt(3)-'0'));
}

/*
timing based, full left/right actuated
send 0 or 1 to signify direction to Serial 3
direction is put into the front angle and
the current angles are propergated down the module
the end angle propergated out to the next modual
through Serial 2 - but inverted to account for 
the alternating actuator positions

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
*/

///#define Serial3 Serial

// current angles of joints
//0 & 1 relay angle 3:undefined for initlal start
char angleArray[] = {'3','3','3','3','3'};

void setup(){
  //motor pin
  pinMode(2,OUTPUT);
  analogWrite(2,0);
  //hydra valves
  for(int i=3;i<13;i++){
    pinMode(i,OUTPUT);
    analogWrite(i,0);
  }
  
  delay(500);
  
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial.begin(115200);
}

void loop(){
  unsigned long prevMessageTime = 0;
  char headAngle = '3';
  char tailAngle = '3';
  
  if(Serial3.available()>0){
    headAngle = Serial3.read();
    
    if(headAngle != '0' && headAngle != '1'){
      return;
    }
    
    /*
    //display state to led 13(debugging only)
    analogWrite(13, 255);
    delay(100);
    analogWrite(13, 0);
    if(headAngle == '1'){
      analogWrite(13, 255);
    }
    */
    
    //read new angle and propergate it down
    //and turn on actuators and motor
    tailAngle = propergate(headAngle);
    //invert angle to account for alternating actuators
    if(tailAngle == '0'){
      Serial2.write('1');
    }
    else if(tailAngle == '1'){
      Serial2.write('0');
    }
    
    //allow time for actuators to move
    delay(1200);
    /*int i = 0;
    while(isMoving() || i<10){
      delay(10);
      i++;
    }*/
    
    //turn off all actuators
    for(int i=0;i<5;i++){
      analogWrite(2*i+3, 0);
      analogWrite(2*i+4, 0);
    }
    //turn off motor
    analogWrite(2, 0);
    
    //return ready signal
    Serial3.write('1');
    
    Serial2.flush();
    Serial3.flush();
    Serial.flush();
    prevMessageTime = millis();
    
    //turn off motors
    analogWrite(2, 0);
  }
  
}

char propergate(char headAngle){
  char endAngle;
  //new array of next angles
  char nextAngleArray[5];
  //new angle is a new angle at front and the rest propergate
  nextAngleArray[0] = headAngle;
  for(int i=1;i<5;i++){
    nextAngleArray[i] = angleArray[i-1];
  }
  
  
  
  //move actuators
  for(int i=0;i<5;i++){
    //if next angle is different from current, move actuator
    if(nextAngleArray[i] != angleArray[i]){
      //adjust for the pin numbers and alternating actuators
      
      //turn on motor
      analogWrite(2, 110);
      
      if(nextAngleArray[i] == '0'){
        analogWrite(2*(4-i)+3+(4-i)%2, 255);
      }
      else if(nextAngleArray[i] == '1'){
        analogWrite(2*(4-i)+4-(4-i)%2, 255);
      }
    }
  }
  
  //move nextAngleArray to angleArray
  endAngle = angleArray[4];
  for(int i=0;i<5;i++){
    angleArray[i] = nextAngleArray[i];
  }
  
  //return angle that gets propergated out
  return endAngle;
}

int sensorArray[5];
boolean isMoving(){
  boolean moving = false;
  
  for(int i=0;i<5;i++){
    if(abs(analogRead(i+1)-sensorArray[i]) > 3){
      moving = true;
    }
    sensorArray[i] = analogRead(i+1);
  }
  
  return moving;
}

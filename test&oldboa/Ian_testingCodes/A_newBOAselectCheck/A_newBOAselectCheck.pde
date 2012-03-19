#include "boashield_pins.h"

void setup(){
  Serial.begin(115200);
/*  for(int i = 2; 2<=13; i++){
    pinMode(i,OUTPUT);
    digitalWrite(i,0);
  }*/
  Serial.println("press 'p' to test");
  boolean gogo = false;
  while(gogo==false){
    if(Serial.read() == 'p'){
      gogo = true;
    }
  }
  Serial.println("main");
}

boolean sinewave = false;

void loop(){
  if(Serial.available()){
    byte tmp = Serial.read();
    if(tmp == 'a'){
      analogWrite(VERT_ACTUATOR_1,0);
      digitalWrite(VERT_ACTUATOR_CTRL_1,HIGH);
      analogWrite(VERT_ACTUATOR_1,250);
      analogWrite(MOTOR_CONTROL,140);
    }
    if(tmp == 'z'){
      analogWrite(VERT_ACTUATOR_1,0);
      digitalWrite(VERT_ACTUATOR_CTRL_1,LOW);
      analogWrite(VERT_ACTUATOR_1,250);
      analogWrite(MOTOR_CONTROL,140);
    }
    if(tmp == 'q'){
      analogWrite(VERT_ACTUATOR_1,0);
      analogWrite(MOTOR_CONTROL,0);
    }
    if(tmp == 'V'){
      int batIN = analogRead(BAT_LEVEL_24V);
      int batREAL = map(batIN,0,1023,0,25000);
      Serial.print(batREAL);
      Serial.println(" mv current battery voltage");
    }
    if(tmp == 's'){
      sinewave = !sinewave;
    }
  }//END OF TMP read
}

#include "boashield_pins.h"

void setup(){
  pinMode(MOTOR_CONTROL,OUTPUT);

  Serial.begin(115200);
  Serial.println("go");
}

void loop(){
  if(Serial.available()){
    byte tmp = Serial.read();  
    if(tmp == 'i'){
      int batIN = analogRead(BAT_LEVEL_24V);
      int batREAL = map(batIN,0,1023,0,25000);
      Serial.print(batREAL);
      Serial.println(" mv current battery voltage");
    }
  }
}

/*This test code changes only motor values
use the serial monitor to change motor speed
i100 should write a value of 100 to the motor
not sure what will happen if you type i5000
*/
//we had solenoid switching going on also but it got quite warm
#include "boashield_pins.h"

void setup(){
  pinMode(MOTOR_CONTROL,OUTPUT);

  Serial.begin(115200);
  Serial.println("go");
}

void loop(){
//  digitalWrite(3,255);
  //digitalWrite(4,0);
//  delay(1000);
//  digitalWrite(3,0);
  //digitalWrite(4,255);
  //delay(1000);

  if(Serial.available()){
    byte tmp = Serial.read();  
      if(tmp == 'i'){        
        analogWrite(MOTOR_CONTROL,150);
        Serial.println("motor 150");
        digitalWrite(VERT_ACTUATOR_CTRL_1,HIGH);
        analogWrite(VERT_ACTUATOR_1,200);
        delay(1000);
        analogWrite(MOTOR_CONTROL,200);
        Serial.println("motor 200");
        digitalWrite(VERT_ACTUATOR_CTRL_1,LOW);
        //valve select switches. that is all
        delay(1000);
        analogWrite(MOTOR_CONTROL,0);
        Serial.println("motor 000");
        analogWrite(VERT_ACTUATOR_1,0);
    }
    if(tmp == 'B'){
      int batIN = analogRead(BAT_LEVEL_24V);
      int batREAL = map(batIN,0,1023,0,25000);
      Serial.print(batREAL);
      Serial.println(" mv current battery voltage");
    }
  }
  
}

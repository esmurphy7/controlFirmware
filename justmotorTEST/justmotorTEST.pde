/*This test code changes only motor values
use the serial monitor to change motor speed
i100 should write a value of 100 to the motor
not sure what will happen if you type i5000
*/
//we had solenoid switching going on also but it got quite warm
void setup(){
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
 // pinMode(2,OUTPUT);
  //digitalWrite(2,0);
  Serial.begin(115200);
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
      char tmp2[3];
      for(int i=0;i<3;i++){
        tmp2[i]=Serial.read();
      }
      byte tmp3=atoi(tmp2);
      Serial.print("atoi output");
      Serial.print(" ");
      Serial.print(tmp3,DEC);
      Serial.println(" ");
      digitalWrite(2,tmp3);
    }
  }
}

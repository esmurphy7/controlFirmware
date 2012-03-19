int SPEED = 100;
const int MOTOR = 0;
boolean ditherValve = true;

void setup(){
    Serial.begin(115200);
    Serial.println("starting");
}
//12 is out apparently
//11 is down apparently
void loop(){
  
  if(Serial.available()){
    byte tmp = Serial.read();
    Serial.println(tmp);
    if(tmp == 'a'){
      SPEED+=10;
      Serial.println(SPEED);
    }
    if(tmp == 'z'){
      SPEED-=10;
      Serial.println(SPEED);
    }
    if(tmp == 'd') {
      analogWrite(2,150);
      analogWrite(11,SPEED);
      analogWrite(12,0);
      delay(500);
      analogWrite(11,SPEED-10);
      delay(2000);
    }
    if(tmp == 'u') {
      analogWrite(2,150);
      analogWrite(11,0);
      analogWrite(12,SPEED);
      delay(500);
      analogWrite(12,SPEED-10);
      delay(2000);
    }
  }
  float sineOut = sin(millis()/1000.0*PI*2.0*200.0);
  byte sineOutput;
  if(sineOut > 1){
    sineOutput = 255;
  }
  else {
    sineOutput = 0;
  }
  analogWrite(2,0);//motor pin
  if(ditherValve) {
    analogWrite(11,sineOutput);//down
    analogWrite(12,0);
  }
/*  delay(1000);
  analogWrite(2,0);//stop
  analogWrite(11,0);
  analogWrite(12,0);
  delay(1000);
  analogWrite(2,MOTOR);
  analogWrite(11,0);//up
  analogWrite(12,SPEED);
  delay(1000);
  analogWrite(2,0);//stop
  analogWrite(12,0);
  analogWrite(11,0);*/
}

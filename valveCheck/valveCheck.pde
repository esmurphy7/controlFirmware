int SPEED = 0;
const int MOTOR = 0;

void setup(){
    Serial.begin(115200);
}
//12 is out apparently
//11 is down apparently
void loop(){
  Serial.println("press 'q' to continue");
  boolean gogo = false;
  while(gogo==false){
    if(Serial.read() == 'q'){
      gogo = true;
    }
  }
  analogWrite(2,MOTOR);//motor pin
  analogWrite(11,SPEED);//down
  analogWrite(12,0);
  delay(1000);
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
  analogWrite(11,0);
}

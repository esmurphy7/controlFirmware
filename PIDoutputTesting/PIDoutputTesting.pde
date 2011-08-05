const int sensorPinX = A0;
const int actuatorPinX = 11;//and 12 for down
const int valveSelectX = 36;
const int motorPin = 2;
const int limit_switchX = 26;
byte cur_pid;
int Ain = 90;
int Aout = 120;
int PWMVALUE=50;

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPin,OUTPUT);
  digitalWrite(motorPin,0);

  pinMode(sensorPinX,INPUT);
  pinMode(actuatorPinX,OUTPUT);
  analogWrite(actuatorPinX,0);
  pinMode(valveSelectX,OUTPUT);
  analogWrite(valveSelectX,OUTPUT);
  
  Serial.println("press 'p' to continue");
  boolean gogo = false;
  while(gogo==false){
    if(Serial.read() == 'p'){
      gogo = true;
    }
  }
  cur_pid = getSensor();
  
}
int getSensor(){
  int sensorReading = analogRead(sensorPinX);
  sensorReading = map(sensorReading, 0, 1023, 0, 255);
  return sensorReading;
}

void goBottom() {
  //digitalWrite(actuatorPinX);
  //delay(100);
  while(getSensor()>=Ain){
    analogWrite(actuatorPinX+1,PWMVALUE);
  }
}

void goBack(){//this makes us go to cur_pid value
  //digitalWrite(actuatorPinX);
  //delay(100);
  if(cur_pid<getSensor()){
    while(getSensor()>cur_pid){
      analogWrite(actuatorPinX+1,PWMVALUE);
    }
  }
  if(cur_pid>getSensor()){
    while(getSensor()<cur_pid){
      analogWrite(actuatorPinX,PWMVALUE);
    }
  }
}

void goTop(){
  //digitalWrite(actuatorPinX);
  //delay(100);
  while(getSensor()<=Aout){
    analogWrite(actuatorPinX,PWMVALUE);
  }
}

void loop(){
  if(Serial.available()){
    byte tmp = Serial.read();
    Serial.println(tmp);
    if(tmp=='a') {cur_pid+=10;}
    if(tmp=='z') {cur_pid-=10;}
    if(tmp=='u'){
      int startTime = millis();
      goTop();
      Serial.print("time up = ");
      Serial.println("millis()-startTime");
      startTime=millis();
      goBack();
      Serial.print("return time = ");
      Serial.print("millis()-startTime");
      Serial.print("change in angle = ");
      Serial.println(Aout-cur_pid);
    }
    if(tmp=='d'){
      int startTime = millis();
      goBottom();
      Serial.print("time down = ");
      Serial.println(millis()-startTime);
      startTime=millis();
      goBack();
      Serial.print("return time = ");
      Serial.println(millis()-startTime);
      Serial.print("change in angle = ");
      Serial.println(cur_pid - Ain);
    }
    if(tmp=='t'){
      goTop();
    }
    if(tmp=='b'){
      goBottom();
    }
    if(tmp=='r'){
      goBack();
    }
    if(tmp=='c'){
      Serial.println(cur_pid);
    }
    if(tmp=='s'){
      Serial.println(getSensor());
    }
  }
}

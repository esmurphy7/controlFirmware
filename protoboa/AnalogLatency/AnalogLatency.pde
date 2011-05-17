int timeStart;
int readTime;
int analog[16]; 

void setup(){
  Serial.begin(9800); 
}

void loop(){
  timeStart = micros();
  for(int i=0;i<16;i++){
    analog[i]=analogRead(i);
  }
  readTime = micros() - timeStart;
  Serial.print(readTime*0.001);
  Serial.println(" ms");
  delay(500);
}


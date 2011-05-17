char inByte;
char outByte;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  if(Serial3.available()){
    inByte=Serial3.read();
    Serial.print(inByte);
    delay(10);
  }

  if(Serial.available()){
    outByte=Serial.read();
    Serial3.print(outByte);
    delay(10);
  }
}





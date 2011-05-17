
char inByte;         // incoming serial byte
char outByte;

void setup()
{
  // start serial port at 9600 bps:
  Serial3.begin(9600);
  Serial.begin(9600);
  pinMode(47, OUTPUT); 
  pinMode(46, OUTPUT); 
}

void loop()
{
  if (Serial3.available()) {
    inByte=Serial3.read();
    Serial.print(inByte);
    digitalWrite(46,LOW);
    digitalWrite(47,HIGH);
    delay(50);
    digitalWrite(47,LOW);
    if(Serial.available()){
      outByte=Serial.read();
      Serial.print(outByte);
      digitalWrite(47,LOW);
      digitalWrite(46,HIGH);
      delay(50);
      digitalWrite(46,LOW);
    }
  }
  if(Serial.available()){
    outByte=Serial.read();
    Serial.print(outByte);
    digitalWrite(47,LOW);
    digitalWrite(46,HIGH);
    delay(50);
    digitalWrite(46,LOW);
  }


}










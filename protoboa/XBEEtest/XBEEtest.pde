

char inByte;
char outByte;

void setup() {
  pinMode(50,OUTPUT);
  pinMode(48,OUTPUT);
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  if(Serial3.available()){
    inByte=Serial3.read();
    Serial.print(inByte);
    digitalWrite(48,LOW);
    digitalWrite(50,HIGH);
    digitalWrite(50,LOW);
  }

  if(Serial.available()){
    outByte=Serial.read();
    Serial3.print(inByte);
    digitalWrite(50,LOW);
    digitalWrite(48,HIGH);
    digitalWrite(48,LOW);
  }
}




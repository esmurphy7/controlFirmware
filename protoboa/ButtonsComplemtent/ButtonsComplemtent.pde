int inByte;

void setup(){
    Serial.begin(9600);
    pinMode(22,OUTPUT);
    pinMode(53,OUTPUT);
}


void loop(){
  if(Serial.available()){
   inByte=Serial.read(); 
  }
  if(inByte=='1'){
  digitalWrite(22,HIGH);
  }
    if(inByte=='2'){
  digitalWrite(22,LOW);
  }
  
 /*   if(inByte=='0'){
  digitalWrite(53,HIGH);
  digitalWrite(22,LOW);
  }*/
}

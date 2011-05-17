byte inArray[3];

int A =16;
int D =28;
int P =12;
int pwmChange;

boolean pwmCMD = true;
boolean digitalCMD = true;

int analogArray[A];
int digitalArray[D];
int pwmArray[P];

int tempInt;
int i;


void setup(){
  Serial.begin(115200);
}

void loop(){
//read anlog and digital ins 
  for(i=0;i<A;i++){
    analogArray[i] = analogRead(i);
  }
  for(i=0;i<D;i++){
    analogArray[i+22] = analogRead(i+22);
  }
  
//receive and parse com
if(Serial.available()>2){
  inArray[0] = Serial.read();
  inArray[1] = Serial.read();  
  inArray[2] = Serial.read();  
}

  
}

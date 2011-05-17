
int i;
int inMsg[64];


void setup(){
  
}

void loop(){
    if(Serial.available()>0){
    inMSG[0] = Serial.read();
    i=0;
    if(inMSG[0] == 254){
      i++;
      while(inMSG[i-1] != 255){
        if(Serial.available()>0){
          inMSG[i] = Serial.read();
          i++;
        }
      }
    }
    if(i == 6){
      received = true; 
    }
  }
  
  if(received == true){
    i=0;
    while(inMSG[i] != 255){
      if(inMSG[i] == 254
    }
  }
  
  
}



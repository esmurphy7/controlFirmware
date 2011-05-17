byte message[64];

int timeStart;
int msgLength;
int i;

boolean received = false;
boolean error = false;

void setup(){
  Serial.begin(115200);
}


void loop(){
  if(Serial.available()>0 && received == false){
    i =0;
    if(Serial.read() == 'y'){
      while(true){
        //timeStart = millis();
        if(Serial.available() > 0 ){
          message[i] = Serial.read();
          if(message[i] == 'y'){
            Serial.print("yxz");
            received = false;
            error = true;
            break;
          }
          if(message[i] == 'z'){
            received = true;
            msgLength = i; // this is the max index, so actual length is +1
            break;
          }
          if(message[i] == 'x'){
            received = true;
            break; 
          }
          i++;
          if(i > 63){
            Serial.print("yxz");
            received = false;
            error = true;
            break;
          }
        }
        /*if((millis() - timeStart) > 2000){
          Serial.print("yxz");
          received = false;
          error = true;
          break;
        }*/
      }
    }
  }
  if(received == true){
    Serial.print('y');
    for(i=0;i<msgLength;i++){
      Serial.print(message[i]);
    }
    Serial.print('z');
    received = false;
  }
  if(error == true){
    error = false;
    for(int k; k<64;k++){ 
      message[k] = 0;
      msgLength = 0;
    }
  }
}


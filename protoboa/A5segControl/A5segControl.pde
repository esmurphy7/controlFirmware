int i = 0;
int inMSG[7];
int sensors[5];

boolean received = false;

void setup(){
  Serial3.begin(115200);

  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(36, OUTPUT);

  pinMode(48, OUTPUT);
  digitalWrite(48,LOW);

  sensors[0] = 126;
  sensors[1] = 126;
  sensors[2] = 126;
  sensors[3] = 126;
  sensors[4] = 126;
}

void loop(){
  if(Serial3.available()>0){
    inMSG[0] = Serial3.read();
    i=0;
    if(inMSG[0] == 254){
      i++;
      while(inMSG[i-1] != 255){
        digitalWrite(48, HIGH);
        if(Serial3.available()>0){
          inMSG[i] = Serial3.read();
          i++;
        }
      }
    }
    if(i == 7){
      for(int k = 0;k<10;k+=2){

        if(126+10 < ((inMSG[k/2+1]))){
          digitalWrite(26+k,LOW);
          digitalWrite(26+k+1,HIGH);
          //analogWrite(2+i,inMSG[i+1]);
        }
        if(126-10 > ((inMSG[k/2+1]))){
          digitalWrite(26+k,HIGH);
          digitalWrite(26+k+1,LOW);
          //analogWrite(2+i,inMSG[i+1]);
        }
        if(126-10 <= ((inMSG[k/2+1])) && 126+10 >= ((inMSG[k/2+1]))){
          digitalWrite(26+k,LOW);
          digitalWrite(26+k+1,LOW);
          //analogWrite(2+i,inMSG[i+1]);
        }
        digitalWrite(48,LOW);
      }
      Serial3.write(254);
      Serial3.write(inMSG[1]);
      Serial3.write(inMSG[2]);
      Serial3.write(inMSG[3]);
      Serial3.write(inMSG[4]);
      Serial3.write(inMSG[5]);  
      Serial3.write(255);  
    }
  }

  /*
  sensors[0] = analogRead(0)>>2;
   sensors[1] = analogRead(1)>>2;
   sensors[2] = analogRead(2)>>2;
   sensors[3] = analogRead(3)>>2;
   sensors[4] = analogRead(4)>>2;
   
   
   Serial3.write(254);
   Serial3.write(sensors[0]);
   Serial3.write(sensors[1]);
   Serial3.write(sensors[2]);
   Serial3.write(sensors[3]);
   Serial3.write(sensors[4]);  
   Serial3.write(255);
   */
}






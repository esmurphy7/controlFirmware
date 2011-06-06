int i = 0;
int inMSG[7];
int sensors[5];
int rightCal[3];
int leftCal[3];

boolean calibrated = false;
boolean received = false;

void setup(){
  Serial.begin(9800);

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

  sensors[0] = 126;
  sensors[1] = 126;
  sensors[2] = 126;
  sensors[3] = 126;
  sensors[4] = 126;
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

      if(i == 7){
        if(calibrated == false){
          calibrate();
        }
        sensors[0] = map(analogRead(2),0,100,0,253);
        sensors[1] = map(analogRead(8),0,100,0,253);
        sensors[2] = map(analogRead(12),0,100,0,253);

        for(int k = 0;k<10;k+=2){
          //sensors[k/2] = map(sensors[k/2],leftCal[k/2],rightCal[k/2],0,253);

          if(sensors[k/2] < inMSG[k/2+1]-10){
            digitalWrite(26+k,LOW);
            digitalWrite(26+k+1,HIGH);
            //analogWrite(2+i,inMSG[i+1]);
          }
          else if(sensors[k/2] > inMSG[k/2+1]+10){
            digitalWrite(26+k,HIGH);
            digitalWrite(26+k+1,LOW);
            //analogWrite(2+i,inMSG[i+1]);
          }
          else{
            digitalWrite(26+k,LOW);
            digitalWrite(26+k+1,LOW);
            //analogWrite(2+i,inMSG[i+1]);
          }
        }
        Serial.write(254);
        Serial.write(sensors[0]);
        Serial.write(sensors[1]);
        Serial.write(sensors[2]);
        Serial.write(sensors[3]);
        Serial.write(sensors[4]);  
        Serial.write(255); 
      }
    }
  }
}

void calibrate(){
  delay(5000);
  digitalWrite(26,HIGH);
  digitalWrite(28,HIGH);
  digitalWrite(30,HIGH);
  delay(2000);
  leftCal[0] = map(analogRead(2),0,1024,0,200);
  leftCal[1] = map(analogRead(8),0,1024,0,200);
  leftCal[2] = map(analogRead(12),0,1024,0,200);
  digitalWrite(26,LOW);
  digitalWrite(28,LOW);
  digitalWrite(30,LOW);
  digitalWrite(27,HIGH);
  digitalWrite(29,HIGH);
  digitalWrite(31,HIGH);       
  delay(2000); 
  rightCal[0] = map(analogRead(2),0,1024,0,200);
  rightCal[1] = map(analogRead(8),0,1024,0,200);
  rightCal[2] = map(analogRead(12),0,1024,0,200);
  digitalWrite(27,LOW);
  digitalWrite(29,LOW);
  digitalWrite(31,LOW);
  leftCal[0] = 20;
  leftCal[1] = 20;
  leftCal[2] = 20;
  rightCal[0] =100;
  rightCal[1] = 100;
  rightCal[2] = 100;

  calibrated = true;
}








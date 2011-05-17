int i;
int k =0;
int tempValue =0;

int strokeLength=500;
int strokeSensor=500;
int tempSensor;

boolean received = false;
boolean standby = true;
boolean respond = false;

char inMSG[6];

void setup(){
  Serial.begin(115200);
  //pinMode(46,OUTPUT);
  pinMode(48,OUTPUT);
  //digitalWrite(46,LOW);
  digitalWrite(48,LOW);
  analogWrite(13,0);
}


void loop(){
  if(Serial.available()>0){
    inMSG[0] = Serial.read();
    i=0;
    if(inMSG[0] == '!'){
      i++;
      while(inMSG[i-1] != '*'){
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
    strokeLength =(inMSG[1]-'0')*1000+(inMSG[2]-'0')*100+(inMSG[3]-'0')*10+(inMSG[4]-'0');
    strokeSensor= strokeLength;
    analogWrite(2,strokeLength);
}
  /*if(strokeLength == 9999){
    respond = true;
  }
  if(strokeLength == 9998){
    standby = true;
    respond = false;
  }
  if(strokeLength == 9997){
    standby = false; 
  }*/
  /*if(k == 10){
   strokeSensor = floor(tempValue/10);
   tempValue = 0;
   k=0;
   }*/

  //analogRead(0);
  //k++;
  //tempSensor = analogRead(0);
  //if(start == true){
  //if(standby == false){
    /*if(strokeSensor > ((strokeLength)+15)){
      digitalWrite(46,HIGH); 
      digitalWrite(48,LOW);
    }
    if(strokeSensor < ((strokeLength)-15)){
      digitalWrite(46,LOW); 
      digitalWrite(48,HIGH);
    }
    if(strokeSensor >= ((strokeLength)-15) && strokeSensor <= ((strokeLength)+15)){
      digitalWrite(46,LOW); 
      digitalWrite(48,LOW);
    }*/
  //}
  /*if(standby == true){
    digitalWrite(46,LOW); 
    digitalWrite(48,LOW);   
  }
  //}
  /*if(start == false){
   digitalWrite(46,LOW); 
   digitalWrite(48,LOW);
   }*/
//  if(respond == true){
    Serial.print('!');
    number_print(strokeSensor);
    Serial.print('*');
 // }
}

void number_print(int number){
  //prints zeros in order to pad the int to 4 characters, then sends the number, currently only does serial 3,
  // final version will handle 1and 2.
  if(number<1000){
    Serial.print(0);
  }
  if(number<100){
    Serial.print(0);
  }
  if(number<10){
    Serial.print(0); 
  }
  Serial.print(number);
}










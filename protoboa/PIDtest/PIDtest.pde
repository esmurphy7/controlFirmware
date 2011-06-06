int i;
int k =0;
int tempValue =0;

int strokeLength=500;
int strokeSensor=500;
int errorSignal =0;
int tempSensor;

float P=2;
float D=0;

int error=0;
int oldError=0;
int dError =0;

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
    strokeSensor= analogRead(2);

    error = abs(strokeLength - strokeSensor);
    dError = error-oldError;
    errorSignal = P*error-D*dError;
    if(errorSignal > 255){
      errorSignal = 255; 
    }
    if(strokeLength < strokeSensor-15){
      analogWrite(9,0);
      analogWrite(8,255);
    }
    else if(strokeLength > strokeSensor+15){
      analogWrite(8,0);
      analogWrite(9,255); 
    }
    else{
      analogWrite(9,0);
      analogWrite(8,0);
    }
    oldError = error;

  }
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












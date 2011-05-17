int i = 0;
int inMSG[7];
int sensors[5];

int leftCal[5];
int rightCal[5];

float error;
const float P=1;


const float DEAD_ZONE = 10;

boolean received = false;
boolean calibrated = false;

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
  pinMode(A2, INPUT);
  pinMode(A8, INPUT);
  pinMode(A12,INPUT);

  pinMode(48, OUTPUT);
  digitalWrite(48,LOW);


  sensors[0] = 226;
  sensors[1] = 226;
  sensors[2] = 226;
  sensors[3] = 226;
  sensors[4] = 226;
}

void loop(){
  if(Serial3.available()>0){
    inMSG[0] = Serial3.read();
    i=0;
    if(inMSG[0] == 254){
      digitalWrite(48, HIGH);
      i++;
      while(inMSG[i-1] != 255){
        if(Serial3.available()>0){
          inMSG[i] = Serial3.read();
          i++;
        }
      }
    }
    if(i == 7){
      if(inMSG[1] == 254 && inMSG[2] == 0){
        digitalWrite(26,HIGH);
        digitalWrite(28,HIGH);
        digitalWrite(30,HIGH);
        delay(1500);
        rightCal[0] = analogRead(2);
        rightCal[1] = analogRead(8);
        rightCal[2] = analogRead(12);
        digitalWrite(26,LOW);
        digitalWrite(28,LOW);
        digitalWrite(30,LOW);
        digitalWrite(27,HIGH);
        digitalWrite(29,HIGH);
        digitalWrite(31,HIGH);       
        delay(1500); 
        leftCal[0] = analogRead(2);
        leftCal[1] = analogRead(8);
        leftCal[2] = analogRead(12);
        digitalWrite(27,LOW);
        digitalWrite(29,LOW);
        digitalWrite(31,LOW);
        calibrated = true;
      } 
      if(calibrated == true){
        sensors[0] = map(analogRead(2),leftCal[0],rightCal[0],0,200);
        sensors[1] = map(analogRead(8),leftCal[1],rightCal[1],0,200);
        sensors[2] = map(analogRead(12),leftCal[2],rightCal[2],0,200);
        for(int k = 0;k<10;k+=2){
          error = -P*(sensors[k/2]+inMSG[k/2+1]);
          if( DEAD_ZONE < error){
            digitalWrite(26+k,LOW);
            digitalWrite(26+k+1,HIGH);
            //continue;
            //analogWrite(2+i,inMSG[i+1]);
          }
          if(-DEAD_ZONE > error){
            digitalWrite(26+k,HIGH);
            digitalWrite(26+k+1,LOW);
            //continue;
            //analogWrite(2+i,inMSG[i+1]);
          }
          if( DEAD_ZONE >= error && -DEAD_ZONE <= error){
            digitalWrite(26+k,LOW);
            digitalWrite(26+k+1,LOW);
            //continue;
            //analogWrite(2+i,inMSG[i+1]);
          }
        }
      }
      digitalWrite(48,LOW);
      Serial3.write(254);
      Serial3.write(sensors[0]);
      Serial3.write(sensors[1]);
      Serial3.write(sensors[2]);
      Serial3.write(sensors[3]);
      Serial3.write(sensors[4]);  
      Serial3.write(255);   
    } 
  }
}











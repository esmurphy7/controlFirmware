int limitsON =0;
int angle = 0;
int throttle =0;

void setup(){
  Serial3.begin(115200);

  pinMode(2,OUTPUT);
  analogWrite(2,0);
  digitalWrite(28,HIGH);
}

void loop(){
  limitsON=0;
  angle=0;
  throttle = map(analogRead(3),0,670,0,255);
  analogWrite(2,throttle);

  if(digitalRead(39) == HIGH){
    angle=270;
    limitsON++; 
  }
  if(digitalRead(41) == HIGH){
    angle=180;
    limitsON++; 
  } 
  if(digitalRead(43) == HIGH){
    angle=90;
    limitsON++;
    if(limitsON ==1 && angle ==0){
      angle+=360;
    }
  } 
  if(digitalRead(45) == HIGH){
    angle=0;
    limitsON++; 
  } 
  angle = angle/limitsON;
  throttle = map(throttle,0,255,1500,500);

  if(digitalRead(37)==HIGH){
    if((angle < 270 && angle > 90) && angle != -1){
      Serial3.write("s0");
      delay(throttle);
    }
    if((angle < 90 || angle > 270)&& angle != -1){
      Serial3.write("s1");
      delay(throttle);
    }

    if(digitalRead(28) == LOW){
      Serial3.write("c12");
      delay(3000);
    }
  }
  angle = -1;
}











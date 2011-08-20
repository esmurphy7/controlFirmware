//to organize A_messagePump better
//Ian split the functions into different files

unsigned long BcheckTIME;
//remember, it's in mV
int batDIE=21000;

int checkBat(){
  int batIN = analogRead(BAT_LEVEL_24V);
  int batREAL = map(batIN,0,1023,0,25000);
  if(batREAL < batDIE){
    //whatever we do to trigger kill switches
    //digitalWrite(one of the kill switches,HIGH);
    interface.sendState(STATE_DEAD,batREAL,batDIE,PAD,PAD,PAD,PAD);
  }
  return batREAL;
}

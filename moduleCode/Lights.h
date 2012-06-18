/*
controls 5 LED's
3 modes 
off, on, propergate and random
 */
#ifndef Lights_h
#define Lights_h

#include <Arduino.h>
#include "titanoboa_pins.h"

const int LED_PERIOD = 20;

//class outputs PID control
//output constraint to between -255 and 255
class Lights{
private:
  //variables
  boolean lightArray[5];
  //when to turn on/off lights by, for ramping
  //set to zero for no ramp
  unsigned long lightTimingArray[5];
  //record of current power, keep under LED_PERIOD
  int lightPowerArray[5];
  //used to pwm LED's
  unsigned long pwmTimer;
  int mode;

public:
  //constructor
  Lights(int newMode = 0){
    //set up defalt
    mode = newMode;
    for(int i = 0;i<5;i++){
      lightArray[i] = false;
      lightTimingArray[i] = 0;
      lightPowerArray[i] = 0;
    }
  }

  void update(){
    switch(mode){
    case 0:
      //turn off
      for(int i=0;i<5;i++){
        lightArray[i] = false;
      }
      break;

    case 1:
      //just turn all on
      for(int i=0;i<5;i++){
        lightArray[i] = true;
      }
      break;
    case 2:
      //use propergate lights
      //pushes handled by other
      break;
    case 3:
      //random
      for(int i=0;i<5;i++){
        if(lightTimingArray[i]<millis()){
          lightTimingArray[i] = millis() + random(500,5000);
          lightArray[i] = !lightArray[i];
          digitalWrite(LED[i],lightArray[i]);
        }
      }
      break;
    default:
      //do nothing
      break;
    }
    //write the output
    for(int i=0;i<5;i++){
      digitalWrite(LED[i],lightArray[i]);
    }
  }

  //push in new on/off state and returns what's pushed out end
  boolean push(boolean headLight){
    boolean tailLight = lightArray[4];
    for(int i=4;i>0;i--){
      lightArray[i] = lightArray[i-1];
    }
    lightArray[0] = headLight;
    return(tailLight);
  }

  void changeMode(int newMode){
    mode = newMode;
  }

  void LightsPWM(){
    //to be added
  }


};


#endif


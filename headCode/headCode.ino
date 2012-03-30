/*

  headCode.ino - Controls head of Titanoboa
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA to control the 
  head actuators and by communicating to the main module BoaBox 
  Arduinos to propagat a sequence of angles from head to tail at 
  an interval initiated by an external controller.  Communicates
  with an external controller over Xbee.

*/

//#include "titanoboa_pins.h"

#define INPUT_SERIAL Serial2            // Xbee
#define TAIL_SERIAL Serial3             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging

//Actuator pins, these may be incorrect, but will be adjusted after testing
#define JAW_OPEN    4
#define JAW_CLOSE   5
#define HEAD_RAISE  6
#define HEAD_LOWER  7


/*************************************************************************
 setup(): Initializes serial ports, pins
**************************************************************************/
void setup(){
  USB_COM_PORT.begin(115200);
  Serial1.begin(115200);
  TAIL_SERIAL.begin(115200);
  INPUT_SERIAL.begin(115200);

  for(int i=4;i<=7;i++){
    pinMode(i,OUTPUT);
    analogWrite(i,0);
  }
}


/*************************************************************************
 loop(): main loop, checks for messages from the joystick and relays them
         to the first module
**************************************************************************/
void loop()
{
  char message;
  int actuator;

  if(INPUT_SERIAL.available())
  {
     TAIL_SERIAL.write(INPUT_SERIAL.read());
  }

  if(INPUT_SERIAL.available())
  {
    message = INPUT_SERIAL.read();
    USB_COM_PORT.write(message);
    TAIL_SERIAL.write(message);

    if(message == 'h')
    {
      while(INPUT_SERIAL.available() < 1)
      {
        delay(1);
      }
      TAIL_SERIAL.write(message);

      message = INPUT_SERIAL.read();
      USB_COM_PORT.write(message);
      
      if(message == '0')
      {
        actuator = JAW_OPEN;
      }
      else if(message == '1')
      {
        actuator = JAW_CLOSE;
      }
      else if(message == '2')
      {
        actuator = HEAD_RAISE;
      }
      else if(message == '3')
      {
        actuator = HEAD_LOWER;
      }
      else
      {
        actuator = -1;
      }
      
      if(actuator != -1)
      {
        for(int i=0; i<=255; i++)
        {
          analogWrite(actuator, i);
          delay(4);
        }
        for(int i=255; i>=0; i--)
        {
          analogWrite(actuator, i);
          delay(4);
        }
      }
      INPUT_SERIAL.flush();
    }
  }
}



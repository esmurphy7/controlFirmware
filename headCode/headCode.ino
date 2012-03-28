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

/*************************************************************************
 setup(): Initializes serial ports, pins
**************************************************************************/
void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  for(int i=4;i<=7;i++){
    pinMode(i,OUTPUT);
    analogWrite(i,0);
  }
}
//normally Serial 3
#define INPUT_SERIAL Serial3


/*************************************************************************
 loop(): main loop, checks for messages from the joystick and relays them
         to the first module
**************************************************************************/
void loop()
{
  char message;
  int actuator;

  if(Serial2.available())
  {
    INPUT_SERIAL.write(Serial2.read());
  }

  if(INPUT_SERIAL.available())
  {
    message = INPUT_SERIAL.read();
    Serial.write(message);
    Serial2.write(message);

    if(message == 'h')
    {
      while(INPUT_SERIAL.available()<1)
      {
        delay(1);
      }
      Serial2.write(message);

      message = INPUT_SERIAL.read();
      Serial.write(message);
      
      if(message == '0')
      {
        actuator = 4;
      }
      else if(message == '1')
      {
        actuator = 5;
      }
      else if(message == '2')
      {
        actuator = 6;
      }
      else if(message == '3')
      {
        actuator = 7;
      }
      else
      {
        actuator = -1;
      }
      
      if(actuator != -1)
      {
        for(int i=0;i<=255;i++)
        {
          analogWrite(actuator,i);
          delay(4);
        }
        for(int i=255;i>=0;i--)
        {
          analogWrite(actuator,i);
          delay(4);
        }
      }
      INPUT_SERIAL.flush();
    }
  }
}



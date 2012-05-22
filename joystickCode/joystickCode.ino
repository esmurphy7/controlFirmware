/*

  joystickCode.ino - Sends wireless commands to Titanoboa
  
  Created: August 2011 
  Part of the titanaboa.ca project
  
  Description: This file needs commenting and a description.

*/

#define XBEE_SERIAL Serial3
#define JOYSTICK_LEFT_PIN 41
#define JOYSTICK_RIGHT_PIN 45
#define JOYSTICK_UP_PIN 39     // TODO: Confirm pin number
#define JOYSTICK_DOWN_PIN 43   // TODO: Confirm pin number
#define JOYSTICK_BUTTON 37
#define CALIBRATE_BUTTON 28
#define THROTTLE_ANALOG_PIN 3

void setup()
{
  XBEE_SERIAL.begin(115200);
  pinMode(2,OUTPUT);
  analogWrite(2,0);
  
  // Internal pull up on the calibrate button
  digitalWrite(28,HIGH);
}

void loop()
{ 
  // Position of throttle potentiometer determines the delay between 
  // angle propegations
  int throttle = map(analogRead(THROTTLE_ANALOG_PIN),0,670,0,255);
  analogWrite(2,throttle);
  throttle = map(throttle,0,255,1000,250);
  
  // Only send commands when joystick button is pressed
  if(digitalRead(JOYSTICK_BUTTON) == HIGH)
  {

    if(digitalRead(CALIBRATE_BUTTON) == LOW)
    {
      XBEE_SERIAL.write("c12");
      delay(3000);
    }
   
    if(digitalRead(JOYSTICK_LEFT_PIN) == HIGH)
    {
      XBEE_SERIAL.write("s0");
      delay(throttle);
    } 
  
    if(digitalRead(JOYSTICK_RIGHT_PIN) == HIGH)
    {
      XBEE_SERIAL.write("s1");
      delay(throttle);
    }
  }
}










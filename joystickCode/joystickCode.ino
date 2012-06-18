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
#define JAW_OPEN 47
#define JAW_CLOSE 49

// Global variables
boolean joystickButtonWasPressed = false;
boolean angleSent = false;

unsigned long waitTill = 0; // What does this do? Please comment.

/*************************************************************************
 * setup(): Initializes serial ports, pins
 **************************************************************************/
void setup()
{
  XBEE_SERIAL.begin(115200);

  // Internal pull up on the calibrate button
  digitalWrite(CALIBRATE_BUTTON, HIGH);
}//end setup()


/*************************************************************************
 * loop(): main loop, sends messages to the headbox from the joystick
 **************************************************************************/
void loop()
{  
  // Only send commands when joystick button is pressed
  if(digitalRead(JOYSTICK_BUTTON) == HIGH)
  {
    joystickButtonWasPressed = true;

    //signal to open jaw
    if(digitalRead(JAW_OPEN)==true)
    {
      XBEE_SERIAL.write("h0");
      waitTill = millis() + 1000;
    }
    else if(digitalRead(JAW_CLOSE)==true)
    {
      XBEE_SERIAL.write("h1");
      waitTill = millis() + 1000;
    }


    // Position of throttle potentiometer determines the delay between 
    // angle propegations
    unsigned int throttle = map(analogRead(THROTTLE_ANALOG_PIN),0,670,1000,250);

    if(digitalRead(CALIBRATE_BUTTON) == LOW)
    {
      XBEE_SERIAL.write("c12");
      waitTill = millis() + 3000;
    }

    if(digitalRead(JOYSTICK_LEFT_PIN) == HIGH)
    {
      // send left angle
      XBEE_SERIAL.write("s0");

      if (angleSent)
      {
        // send propagation delay
        XBEE_SERIAL.write(lowByte(throttle));
        XBEE_SERIAL.write(highByte(throttle));
      }
      else
      {
        XBEE_SERIAL.write((byte)0);
        XBEE_SERIAL.write((byte)0);
        angleSent = true;
      }
      waitTill = millis() + throttle;
    }
    else if (digitalRead(JOYSTICK_RIGHT_PIN) == HIGH)
    {
      // send right angle
      XBEE_SERIAL.write("s1");
      if (angleSent)
      {
        // send propagation delay
        XBEE_SERIAL.write(lowByte(throttle));
        XBEE_SERIAL.write(highByte(throttle));
      }
      else
      {
        XBEE_SERIAL.write((byte)0);
        XBEE_SERIAL.write((byte)0);
        angleSent = true;
      }
      waitTill = millis() + throttle;
    }
    else
    {
      angleSent = false;
    }

    // What does this do? Please comment.
    while(millis() < waitTill)
    {
      //delay for debouncing
      delay(5);
      if(digitalRead(JOYSTICK_BUTTON) == LOW)
      {
        XBEE_SERIAL.write("k");
        joystickButtonWasPressed = false;
        break;
      }
    }

  }
  else
  {
    if(joystickButtonWasPressed)
    {
      //delay for debouncing
      delay(5);
      if(digitalRead(JOYSTICK_BUTTON) == LOW)
      {
        XBEE_SERIAL.write("k");
        joystickButtonWasPressed = false;
      }
    }
    angleSent = false;
  }

}//end loop()










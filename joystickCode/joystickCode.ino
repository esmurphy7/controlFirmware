/*

 joystickCode.ino - Sends wireless commands to Titanoboa
 
 Created: August 2011 
 Part of the titanaboa.ca project
 
 Description: This file needs commenting and a description.
 
 */

#define XBEE_SERIAL Serial3
#define USB_COM_PORT Serial
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
  USB_COM_PORT.begin(115200);

  // Internal pull up on the calibrate button
  digitalWrite(CALIBRATE_BUTTON, HIGH);

  USB_COM_PORT.println("Hi I'm Titanoboa, ready to rule the world!");
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

      //delay so modules have time to sort last set point but not enough to be noticable
      delay(5);
      //send lights command
      XBEE_SERIAL.print('L');
      XBEE_SERIAL.print(random(0,2)<1 ? '0':'1');
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

      //delay so modules have time to sort last set point but not enought to be noticable
      delay(5);
      //send lights command
      XBEE_SERIAL.print('L');
      XBEE_SERIAL.print(random(0,2)<1 ? '0':'1');
    }
    else
    {
      angleSent = false;
    }

    // This is a hacked together delay() replacement that also monitors the kill switch
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

  // message passing to and from usb
  // Should be careful with this in the future not to send commands that do bad things
  if (USB_COM_PORT.available() > 0)
  {
    XBEE_SERIAL.write(USB_COM_PORT.read());
  }
  if(XBEE_SERIAL.available() > 0)
  {
    USB_COM_PORT.write(XBEE_SERIAL.read());
  }

}//end loop()










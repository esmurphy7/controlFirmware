/*

 joystickCode.ino - Sends wireless commands to Titanoboa
 
 Created: August 2011 
 Part of the titanaboa.ca project
 
 Description: This file needs commenting and a description.
 
 */

#define XBEE_SERIAL Serial3
#define USB_COM_PORT Serial

// Joystick pins
#define JOYSTICK_LEFT_PIN   41
#define JOYSTICK_RIGHT_PIN  45
#define JOYSTICK_UP_PIN     43
#define JOYSTICK_DOWN_PIN   39
#define JOYSTICK_BUTTON     37

#define CALIBRATE_BUTTON 28
#define JAW_OPEN  47
#define JAW_CLOSE 49
#define VERTICAL_STRAIGHTEN 35

// Knobs
#define THROTTLE_PIN     1
#define MOTOR_SPEED_PIN  3
#define RSVD_0_PIN       0
#define RSVD_2_PIN       2
#define RSVD_4_PIN       4


// Global variables
boolean joystickButtonWasPressed = false;
boolean angleSent = false;

int motorSpeed = 200;
byte verticalOnTheFly = true;

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
    int buttonVal;

  // Only send commands when joystick button is pressed
  if(digitalRead(JOYSTICK_BUTTON) == HIGH)
  {
    joystickButtonWasPressed = true;

    // check for motor speed change
    buttonVal = map(analogRead(MOTOR_SPEED_PIN), 0, 1023, 0, 255);
    if (abs(buttonVal - motorSpeed) > 5)
    {
        USB_COM_PORT.print("detected motor speed change, old value: ");
        USB_COM_PORT.print(motorSpeed);
        motorSpeed = buttonVal;
        XBEE_SERIAL.write('m');
        XBEE_SERIAL.write(motorSpeed);
        USB_COM_PORT.print(", new value: ");
        USB_COM_PORT.println(motorSpeed);
    }

        // check for vertical straightening on the fly disable/enable
        buttonVal = analogRead(RSVD_0_PIN);
        if ((buttonVal < 475) && (verticalOnTheFly == true))
        {
            USB_COM_PORT.println("turning off vertical straightening while slithering");
            verticalOnTheFly = false;
            XBEE_SERIAL.write("f0");
        }
        else if ((buttonVal > 525) && (verticalOnTheFly == false))
        {
            USB_COM_PORT.println("turning on vertical straightening while slithering");
            verticalOnTheFly = true;
            XBEE_SERIAL.write("f1");
        }

        //signal to open/close jaw
        if (digitalRead(JAW_OPEN) == true)
        {
            USB_COM_PORT.println("opening jaw");
            XBEE_SERIAL.write("h0");
            waitTill = millis() + 1000;
        }
        else if (digitalRead(JAW_CLOSE) == true)
        {
            USB_COM_PORT.println("closing jaw");
            XBEE_SERIAL.write("h1");
            waitTill = millis() + 1000;
        }

    if (digitalRead(VERTICAL_STRAIGHTEN) == HIGH)
    {
      USB_COM_PORT.print("straighten vertical\n");
      XBEE_SERIAL.write("l");
      delay(1000);
    }

    // Position of throttle potentiometer determines the delay between 
    // angle propagations
    unsigned int throttle = map(analogRead(THROTTLE_PIN), 0, 1023, 1000, 250);

    if(digitalRead(CALIBRATE_BUTTON) == LOW)
    {
      XBEE_SERIAL.write("c");
      // Tell first module it's the first module.
      XBEE_SERIAL.write(1);
      
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
    USB_COM_PORT.print("received message from USB\n");
    if (USB_COM_PORT.read() == 'M')
    {
      // Confirm this wasn't random noise
      if (USB_COM_PORT.read() == 'M')
      {
        // Enter manual mode
        manualControl();
      }
    }
  }

  if(XBEE_SERIAL.available() > 0)
  {
    USB_COM_PORT.write(XBEE_SERIAL.read());
  }

}//end loop()


/**************************************************************************************
  manualControl(): Allows for manual actuator control over the USB_SERIAL_PORT
 *************************************************************************************/
void manualControl()
{
    boolean manual = true;
    char byteIn = 'z';
    int moduleSelect = 0;
    int segSelect = 0;
    boolean motor = false;
    int actuationDelay = 200;
    char delaySetting = 'm';

    displayMenu();

    while(manual == true)
    {
        if(USB_COM_PORT.available() > 0)
        {
            byteIn = USB_COM_PORT.read();

            switch(byteIn)
            {
            case 'c':
                USB_COM_PORT.print("\nNOT YET IMPLEMENTED: Running calibration...\n");
                //calibrate();
                break;

            case 'p':
                readAllButtons();
                break;

            case 'r':
                USB_COM_PORT.print("\nNOT YET IMPLEMENTED: Saving current vertical position\n");
                //saveVerticalPosition();
                break;

            case 'a':
                USB_COM_PORT.print("\nNOT YET IMPLEMENTED: Straightening verticals\n");
                //straightenVertical();
                break;

            case 'm':
            {
                USB_COM_PORT.print("\nNOT YET IMPLEMENTED: module select\n");
/*
                nextChar = USB_COM_PORT.read();
                switch (nextChar)
                {
			    case '1':
			        moduleSelect = 0;
			        USB_COM_PORT.print("Module1\n");
			        break;
			    case '2':
			        moduleSelect = 1;
			        USB_COM_PORT.print("Module2\n");
			        break;
			    case '3':
			        moduleSelect = 2;
			        USB_COM_PORT.print("Module3\n");
			        break;
			    case '4':
			        moduleSelect = 3;
			        USB_COM_PORT.print("Module4\n");
			        break;
			    case '5':
			        moduleSelect = 4;
			        USB_COM_PORT.print("Module5\n");
			        break;
                }
*/
            }
            break;

            case 'v':
            {
                USB_COM_PORT.print("\nNOT YET IMPLEMENTED: vertebrae select\n");
/*
                nextChar = USB_COM_PORT.read();
                switch (nextChar)
                {
                case '1':
                    segSelect = 0;
                    USB_COM_PORT.print("Seg1\n");
                    break;
                case '2':
                    segSelect = 1;
                    USB_COM_PORT.print("Seg2\n");
                    break;
                case '3':
                    segSelect = 2;
                    USB_COM_PORT.print("Seg3\n");
                    break;
                case '4':
                    segSelect = 3;
                    USB_COM_PORT.print("Seg4\n");
                    break;
                case '5':
                    segSelect = 4;
                    USB_COM_PORT.print("Seg5\n");
                    break;
                }
*/
            }
            break;

            case 'd':
            {
                while (USB_COM_PORT.available() < 1);
                delaySetting = USB_COM_PORT.read();
                switch (delaySetting)
                {
                case 's':
                    actuationDelay = 150;
                    USB_COM_PORT.print("Actuation delay set to smallest time (150ms)\n");
                    break;
                case 'm':
                    actuationDelay = 200;
                    USB_COM_PORT.print("Actuation delay set to medium time (200ms)\n");
                    break;
                case 'l':
                    actuationDelay = 300;
                    USB_COM_PORT.print("Actuation delay set to largest time (300ms)\n");
                    break;
                default:
                    actuationDelay = 200;
                    USB_COM_PORT.print("Invalid entry, actuation delay set to medium time (200ms)\n");
                    break;
                }
            }
            break;

            case 'l':
                USB_COM_PORT.print("NOT YET IMPLEMENTED: l dir\n");
                break;

            case 'k':
                USB_COM_PORT.print("NOT YET IMPLEMENTED: k dir\n");
                break;

            case 'i':
                USB_COM_PORT.print("NOT YET IMPLEMENTED: i dir\n");
                break;

            case 'o':
                USB_COM_PORT.print("NOT YET IMPLEMENTED: o dir\n");
                break;

            case 's':
                //StopMov();
                USB_COM_PORT.print("NOT YET IMPLEMENTED: STOPPED\n");
                break;

            case 'e':
                displayMenu();
                break;

            case 'q':
                manual = false;
                break;

            default:
                USB_COM_PORT.print("doesn't do anything\n");
                break;
            }//end switch
        }//end if USB_COM_PORT

        byteIn = 'z';
    }
    USB_COM_PORT.println("\nManual Control mode exited");
}// end manualcontrol


/**************************************************************************************
  displayMenu():
 *************************************************************************************/
void displayMenu()
{
    USB_COM_PORT.print("\nManual control mode menu\n");
    USB_COM_PORT.print("commands: m1-5 to select module\n");
    USB_COM_PORT.print("          v1-5 to select vertebrae\n");
    USB_COM_PORT.print("          k/l (extend/retract) - horizontal actuation\n");
    USB_COM_PORT.print("          i/o (extend/retract) - vertical actuation\n");
    USB_COM_PORT.print("          d* - adjust actuation delay, where *=s(small),m(medium),l(large)\n");
    USB_COM_PORT.print("          c - calibrate\n");
    USB_COM_PORT.print("          p - print raw values of all knobs, buttons, etc\n");
    USB_COM_PORT.print("          r - record current vertical position as straight\n");
    USB_COM_PORT.print("          a - straighten verticals\n");
    USB_COM_PORT.print("          s - stop motor\n");
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n");
}


/**************************************************************************************
  readAllButtons():
 *************************************************************************************/
void readAllButtons()
{
    int buttonVal = 0;

    // Get the current raw value of the throttle
    buttonVal = analogRead(THROTTLE_PIN);
    USB_COM_PORT.print("Throttle: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of the motor speed
    buttonVal = analogRead(MOTOR_SPEED_PIN);
    USB_COM_PORT.print("Motor speed: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of reserved 1 knob
    buttonVal = analogRead(RSVD_0_PIN);
    USB_COM_PORT.print("Reserved 0: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of reserved 2 knob
    buttonVal = analogRead(RSVD_2_PIN);
    USB_COM_PORT.print("Reserved 2: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of reserved 3 knob
    buttonVal = analogRead(RSVD_4_PIN);
    USB_COM_PORT.print("Reserved 4: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of jaw switch
    buttonVal = digitalRead(JAW_OPEN);
    USB_COM_PORT.print("Jaw open: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JAW_CLOSE);
    USB_COM_PORT.print("Jaw close: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of the vertical straighten switch
    buttonVal = digitalRead(VERTICAL_STRAIGHTEN);
    USB_COM_PORT.print("Vertical straighten: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of the calibrate button
    buttonVal = digitalRead(CALIBRATE_BUTTON);
    USB_COM_PORT.print("Calibrate: ");
    USB_COM_PORT.println(buttonVal);
    USB_COM_PORT.println();

    // Get the current raw value of the joystick
    buttonVal = digitalRead(JOYSTICK_LEFT_PIN);
    USB_COM_PORT.print("Joystick left: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_RIGHT_PIN);
    USB_COM_PORT.print("Joystick right: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_UP_PIN);
    USB_COM_PORT.print("Joystick up: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_DOWN_PIN);
    USB_COM_PORT.print("Joystick down: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_BUTTON);
    USB_COM_PORT.print("Joystick button: ");
    USB_COM_PORT.println(buttonVal);
    USB_COM_PORT.println();
}



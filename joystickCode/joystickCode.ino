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
  // Check USB serial for command to enter manaul mode.
  if (USB_COM_PORT.available() > 0)
  {
    if (USB_COM_PORT.read() == 'M')
    {
      manualMode();
    }   
  }
  
  // Read the status of the buttons and switches
  boolean killSwitchPressed = digitalRead(JOYSTICK_BUTTON) == HIGH;
  boolean jawOpenSwitchPressed = digitalRead(JAW_OPEN) == HIGH;
  boolean jawCloseSwitchPressed = digitalRead(JAW_CLOSE) == HIGH;
  boolean straightenVertSwitchPressed = digitalRead(VERTICAL_STRAIGHTEN) == HIGH;
  boolean calibrateButtonPressed = digitalRead(CALIBRATE_BUTTON) == LOW;
  boolean joystickLeftPressed = digitalRead(JOYSTICK_LEFT_PIN) == HIGH;
  boolean joystickRightPressed = digitalRead(JOYSTICK_RIGHT_PIN) == HIGH;
  
  unsigned short motorSpeedKnob = analogRead(MOTOR_SPEED_PIN);
  unsigned short verticalOnTheFlyKnob = analogRead(RSVD_0_PIN);
  unsigned short propagationDelayKnob = analogRead(THROTTLE_PIN);

  // TODO: Send button and switch status 

}//end loop()


/**************************************************************************************
  manualControl(): Allows for manual control over the USB_SERIAL_PORT
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
            case 'p':
                readAllButtons();
                break;

            case 'e':
                displayMenu();
                break;

            case 'q':
                manual = false;
                break;

            default:
                USB_COM_PORT.println("doesn't do anything\n");
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
    USB_COM_PORT.print("commands: p - print raw values of all knobs, buttons, etc\n");
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



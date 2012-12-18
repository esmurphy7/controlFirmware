/*

  headCode.ino - Controls head of Titanoboa
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA to control the 
  head actuators and communicate with the main modules
  to propagate a sequence of angles from head to tail at 
  an interval initiated by an external controller.  Communicates
  with an external controller over Xbee.

*/

#define HEADBOARD2
//#define HEADBOARD1
#if defined(HEADBOARD2) && defined(HEADBOARD1)
	#error "Headboard1 and Headboard2 settings are mutually exclusive"
#endif

//#include "titanoboa_pins.h"
#ifdef HEADBOARD2
#include "titanoboa_headboard_pins.h"
#endif

#define INPUT_SERIAL Serial3            // Xbee
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging



// Actuator pins
#ifdef HEADBOARD2
#define JAW_OPEN	HEAD_ACTUATOR_3 //Yep, if I had just wired individual pwm channels to each valve control, this would look a lot simpler. Bleh. -Mike
#define JAW_CLOSE	HEAD_ACTUATOR_3
#define JAW_CTRL	HEAD_ACTUATOR_CTRL_3
#define JAW_OPEN_CTRL_SELECT	1
#define JAW_CLOSE_CTRL_SELECT	0

#define HEAD_RAISE	HEAD_ACTUATOR_2
#define HEAD_LOWER	HEAD_ACTUATOR_2
#define HEAD_CTRL	HEAD_ACTUATOR_CTRL_2
#define HEAD_RAISE_CTRL_SELECT	0
#define HEAD_LOWER_CTRL_SELECT	1

#define AUX_EXTEND	HEAD_ACTUATOR_1
#define AUX_RETRACT	HEAD_ACTUATOR_1
#define AUX_CTRL	HEAD_ACTUATOR_CTRL_1
#define AUX_EXTEND_CTRL_SELECT	0
#define AUX_RETRACT_CTRL_SELECT	1
#endif
#ifdef HEADBOARD1
#define JAW_CLOSE   3
#define JAW_OPEN    5
#define HEAD_RAISE  6
#define HEAD_LOWER  7
#endif

// Propagation delay storage delays
unsigned int currentTime;
unsigned int lastAngleReceivedTime;
unsigned int timeDifference;
unsigned int throttleDelay;

/*************************************************************************
 setup(): Initializes serial ports, pins
**************************************************************************/
void setup()
{
  USB_COM_PORT.begin(115200);
  TAIL_SERIAL.begin(115200);
  INPUT_SERIAL.begin(115200);
    USB_COM_PORT.println("Hi I'm the Titanoboa Head!");

  // Initialize Actuator Controls Off
 #ifdef HEADBOARD1
 analogWrite(JAW_CLOSE, 0);
  analogWrite(JAW_OPEN, 0);
  analogWrite(HEAD_RAISE, 0);
  analogWrite(HEAD_LOWER, 0);
#endif
  //Initialize new headboard pins, similar outputs to a boashield
#ifdef HEADBOARD2
  pinMode(DITHER, OUTPUT);
  digitalWrite(DITHER, LOW); //Have dither set as low in this case (no actuation at all)

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
  
  for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATOR[i],OUTPUT); //Set all actuator outputs to output, and set them to 0
    analogWrite(HEAD_ACTUATOR[i],0);
  }
    for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATOR_CTRL[i],OUTPUT); //Set all actuator control outputs (selects pairs, digital) to output, 0
	digitalWrite(HEAD_ACTUATOR_CTRL[i], LOW); //Setting to zero selects odd pairs (1,3,5 as opposed to 2,4,6)
  }

  // Initialize position sensor inputs.
  for(int i=0;i<5;i++)
  {
    pinMode(HEAD_POS_SENSOR[i],INPUT); //Set all actuator position sensors to input
  }
#endif
  currentTime = 0;
  lastAngleReceivedTime = 0;
  timeDifference = 0;
}

/*************************************************************************
 loop(): main loop, checks for messages from the joystick and relays them
         to the first module
**************************************************************************/
void loop()
{
  char message;

  // Check for command from USB serial to enter manual control
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

  // Check for commands over XBee
  if(INPUT_SERIAL.available())
  {
    message = INPUT_SERIAL.read();
    USB_COM_PORT.print("received message from xbee: ");
    USB_COM_PORT.print(message);
    USB_COM_PORT.print(" [");
    USB_COM_PORT.print(message, DEC);
    USB_COM_PORT.print("]\n");

        // Kill command
        if (message == 'k')
        {
            analogWrite(JAW_OPEN, 0);
            analogWrite(JAW_CLOSE, 0);
            analogWrite(HEAD_RAISE, 0);
            analogWrite(HEAD_LOWER, 0);
            analogWrite(AUX_EXTEND, 0);
            analogWrite(AUX_RETRACT, 0);
        }

    // Check for new setpoints command.
    if (message == 's')
    {
      processSetpointsMessage();
      return;
    }

    // Check for head/jaw movements command.
    if(message == 'h')
    {
      processHeadJawMessage();
      return;
    }

    // Pass through all other commands to the first module.
    TAIL_SERIAL.write(message);
  }
}//end loop()


/**************************************************************************************
  processSetpointsMessage(): Transmits setpoints for the first module for propegation 
 *************************************************************************************/
void processSetpointsMessage()
{
  currentTime = millis();
  timeDifference = currentTime - lastAngleReceivedTime;
  
  // setpoints to come, wait for all data to arrive, data includes angle and propagation delay
  while(INPUT_SERIAL.available() < 3);
  // get angle
  char message = INPUT_SERIAL.read();
  // throttle delay low byte comes first followed by high byte
  throttleDelay = (INPUT_SERIAL.read());
  throttleDelay += (INPUT_SERIAL.read() << 8);
  
  // throttle delay of 0 means this is the first angle
  if ((timeDifference > (throttleDelay - 20)) && (timeDifference < (throttleDelay + 20)) || throttleDelay == 0)
  {
    // Delay between receiving angles is within tolerance ok to send the modules
    TAIL_SERIAL.write('s');
    TAIL_SERIAL.write(message);
    USB_COM_PORT.print("new setpoint: ");
    USB_COM_PORT.println(message);
  }
  else
  {
    USB_COM_PORT.print("dropped\n\n");
  }
  
  USB_COM_PORT.print("time between: ");
  USB_COM_PORT.print(timeDifference, DEC);
  USB_COM_PORT.print("\tthrottleDelay: ");
  USB_COM_PORT.println(throttleDelay, DEC);
  
  // update last time an angle was received
  lastAngleReceivedTime = currentTime;
  return; 
}

/**************************************************************************************
  processHeadJawMessage(): Uses joystick commands to open the mouth or close the jaw 
 *************************************************************************************/
void processHeadJawMessage()
{
  int actuator;
  char headjawmessage;  
  
  while(INPUT_SERIAL.available() < 1)
  {
    delay(1);
  }
    TAIL_SERIAL.write("hx");  //x is a dummy character because the headboard is expecting 2 characters
  
    headjawmessage = INPUT_SERIAL.read();
    USB_COM_PORT.print("received jaw message: ");
    USB_COM_PORT.println(headjawmessage);
  
  if(headjawmessage == '0')
  {
      actuator = -1;
	  //actuator = JAW_OPEN;
#ifdef HEADBOARD2
		digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);
#endif
		
        for(int i=100; i<=255; i++)
        {
            analogWrite(JAW_OPEN, i);
            delay(10);
        }
        for(int i=255; i>=50; i--)
        {
            analogWrite(JAW_OPEN, i);
            delay(6);
        }
        analogWrite(JAW_OPEN, 0);
		
  }
  else if(headjawmessage == '1')
  {
        actuator = -1;
		//actuator = JAW_CLOSE;
#ifdef HEADBOARD2
		digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);
#endif

        for(int i=0; i<=255; i++)
        {
            analogWrite(JAW_CLOSE, i);
            delay(4);
        }
        for(int i=255; i>=0; i--)
        {
            analogWrite(JAW_CLOSE, i);
            delay(4);
        }

  }
  else if(headjawmessage == '2')
  {
    actuator = HEAD_RAISE;
	#ifdef HEADBOARD2
		digitalWrite(HEAD_CTRL, HEAD_RAISE_CTRL_SELECT);
	#endif
  }
  else if(headjawmessage == '3')
  {
    actuator = HEAD_LOWER;
	#ifdef HEADBOARD2
    digitalWrite(HEAD_CTRL, HEAD_LOWER_CTRL_SELECT);
	#endif
  }
#ifdef HEADBOARD2
  else if(headjawmessage == '4')
  {
    actuator = AUX_EXTEND;
    digitalWrite(AUX_CTRL, AUX_EXTEND_CTRL_SELECT);
  }
    else if(headjawmessage == '5')
  {
    actuator = AUX_RETRACT;
    digitalWrite(AUX_CTRL, AUX_RETRACT_CTRL_SELECT);
#endif
  }
  else
  {
    actuator = -1;
  }
  
  if(actuator != -1) //If we've not attempted to select an invalid actuator...
  {
    for(int i=0; i<=255; i++) //Open the valve bit by bit by ramping PWM out up
    {
      analogWrite(actuator, i);
      delay(4); //TODO: This delay of 4ms may have to be tweaked
    }
    for(int i=255; i>=0; i--) //Close the valve by ramping pwm down
    {
      analogWrite(actuator, i);
      delay(4);
    }
	//actuator = -1; //Resets actuator variable 
  }
  INPUT_SERIAL.flush(); //This may not be the smartest action we can take
  return; 
}

/**************************************************************************************
  manualControl(): Allows for manual actuator control over the USB_SERIAL_PORT
 *************************************************************************************/
void manualControl()
{
  boolean manual = true;
  char byteIn = 'z';
  int actuatorSel = 0;
  int actuationDelay = 200;
  char delaySetting = 'm';
  #ifdef HEADBOARD2
  int led_choice = -1;
  int sensors[6];
  #endif
  
    displayMenu();
  
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      byteIn = USB_COM_PORT.read();
      
      // characters in use: a, b, c, j, h, u, m, d, l, k, s, q
      switch(byteIn)
      {
		#ifdef HEADBOARD2
        case 'a':
          while(USB_COM_PORT.available() < 1)
          {
            delay(1);
          }
        
          led_choice = USB_COM_PORT.read();
          if(led_choice == -1) break; //read() returns -1 if no data is available
          switch(led_choice)
          {
            case '1': digitalWrite(LED_1, HIGH); break; 
            case '2': digitalWrite(LED_2, HIGH); break;
            case '3': digitalWrite(LED_3, HIGH); break;
            case '4': digitalWrite(LED_4, HIGH); break;
            case '5': digitalWrite(LED_5, HIGH); break;
            case '6': digitalWrite(LED_6, HIGH); break;
            default: break;
          }
          USB_COM_PORT.print("LED ");
          USB_COM_PORT.write(led_choice);
          USB_COM_PORT.print(" turned on\n");
          break;
        case 'b':
         while(USB_COM_PORT.available() < 1)
          {
            delay(1);
          }
          led_choice = USB_COM_PORT.read();
          if(led_choice == -1) break; //read() returns -1 if no data is available
          switch(led_choice)
          {
            case '1': digitalWrite(LED_1, LOW); break; 
            case '2': digitalWrite(LED_2, LOW); break;
            case '3': digitalWrite(LED_3, LOW); break;
            case '4': digitalWrite(LED_4, LOW); break;
            case '5': digitalWrite(LED_5, LOW); break;
            case '6': digitalWrite(LED_6, LOW); break;
            default: break;
          }
          USB_COM_PORT.print("LED ");
          USB_COM_PORT.write(led_choice);
          USB_COM_PORT.print(" turned off\r\n");
          break;
        case 'c':
          for(int k =0;k<6;k++)
          {
            sensors[k] = analogRead(HEAD_POS_SENSOR[k]);
            USB_COM_PORT.print("Sensor ");
            USB_COM_PORT.print(k+1,DEC);
            USB_COM_PORT.print(" reads ");
            USB_COM_PORT.print(sensors[k],DEC);
            USB_COM_PORT.print("\r\n");
          }
          USB_COM_PORT.print("\r\n");
          break;
         #endif
        case 'j':
          actuatorSel = 0;
          StopMov();
          USB_COM_PORT.print("Jaw actuator selected\n");
          break;
        case 'h':
          actuatorSel = 1;
          StopMov();
          USB_COM_PORT.print("Head actuator selected\n");
          break;
		  #ifdef HEADBOARD2
        case 'u':
          actuatorSel = 2;
          StopMov();
          USB_COM_PORT.print("Auxiliary actuator selected\n");
          break;
		  #endif

        // new motor speed setting received
        case 'm':
        {
            while (INPUT_SERIAL.available() < 1);
            int ms = INPUT_SERIAL.read();
            TAIL_SERIAL.print('m');
            TAIL_SERIAL.print(ms);
            USB_COM_PORT.print("new motor speed received: ");
            USB_COM_PORT.print(ms, DEC);
            USB_COM_PORT.println();
            break;
        }

        case 'd':
          StopMov();
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
          break;
          
        case 'l':
          StopMov();
          //instruct downsteam module to turn on motor, will run for 200ms
          TAIL_SERIAL.write('g');
          USB_COM_PORT.print("l dir\n");
          if (actuatorSel == 0)
          {
			#ifdef HEADBOARD2
				digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);
            #endif
			analogWrite(JAW_CLOSE, 255);
          }
          else if (actuatorSel == 1)
          {
			#ifdef HEADBOARD2
				digitalWrite(HEAD_CTRL, HEAD_LOWER_CTRL_SELECT);
            #endif
			analogWrite(HEAD_LOWER, 255);
          }
		  #ifdef HEADBOARD2
			  else if(actuatorSel == 2)
			  {
				digitalWrite(AUX_CTRL, AUX_EXTEND_CTRL_SELECT);
				analogWrite(AUX_EXTEND, 255);
			  }
		  #endif
          delay(400);
          StopMov();
          break;
         
        case 'k':
          StopMov();
          //instruct downsteam module to turn on motor, will run for 200ms
          USB_COM_PORT.print("sending motor on\n");
          TAIL_SERIAL.write('g');
          USB_COM_PORT.print("k dir\n");
          if (actuatorSel == 0)
          {
			#ifdef HEADBOARD2
				digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);
            #endif
			analogWrite(JAW_OPEN, 255);
          }
          else if (actuatorSel == 1)
          {
			#ifdef HEADBOARD2
				digitalWrite(HEAD_CTRL, HEAD_RAISE_CTRL_SELECT);
            #endif
			analogWrite(HEAD_RAISE, 255);
          }
          else if(actuatorSel == 2)
          {
			#ifdef HEADBOARD2
				digitalWrite(AUX_CTRL, AUX_RETRACT_CTRL_SELECT);
            #endif
			analogWrite(AUX_RETRACT, 255);
          }
          delay(400);
          StopMov();
          USB_COM_PORT.print("done, motor can turn off\n");
          break;
        
        case 's':
          StopMov();
          USB_COM_PORT.print("STOPPED\n");
          break;

        case 'e':
            displayMenu();
            break;

        case 'q':
          manual = false;
          break;
      }//end switch
    }//end if serial
    
    byteIn = 'z';
  }
  USB_COM_PORT.print("\nManual Control mode exited\n");
}


/**************************************************************************************
  displayMenu():
 *************************************************************************************/
void displayMenu()
{
    USB_COM_PORT.print("\nManual control mode menu\n");
    USB_COM_PORT.print("commands: j or h to select jaw or head actuator.");
#ifdef HEADBOARD2
    USB_COM_PORT.print("u to select Aux actuator\n");
#endif
#ifdef HEADBOARD1
    USB_COM_PORT.print("\n");
#endif
#ifdef HEADBOARD2
    USB_COM_PORT.print("          a/b - leds [0-6]\n");
    USB_COM_PORT.print("          c - list sensor values\n");
#endif
    USB_COM_PORT.print("          k/l - actuation\n");
    USB_COM_PORT.print("          d'v' - adjust actuation delay, where v=s(small),m(medium),l(large)\n");
    USB_COM_PORT.print("          s - stop motor\n");
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n\n");
}


/**************************************************************************************
  StopMov(): In manual mode, stops movement of all actuators.
 *************************************************************************************/
void StopMov()
{
  analogWrite(JAW_OPEN, 0);
  analogWrite(JAW_CLOSE, 0);
  analogWrite(HEAD_RAISE, 0);
  analogWrite(HEAD_LOWER, 0);
  #ifdef HEADBOARD2
	analogWrite(AUX_EXTEND, 0);
	analogWrite(AUX_RETRACT, 0);
  #endif
  return;
}


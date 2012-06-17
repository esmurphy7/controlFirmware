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

#define INPUT_SERIAL Serial3            // Xbee
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging

// Actuator pins, these may be incorrect, but will be adjusted after testing
#define JAW_CLOSE   3
#define JAW_OPEN    5
#define HEAD_RAISE  6
#define HEAD_LOWER  7

const char myModuleNumber = 0;

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
  Serial1.begin(115200);
  TAIL_SERIAL.begin(115200);
  INPUT_SERIAL.begin(115200);
  
  // Print out the loaded cabibration and angle array.
  USB_COM_PORT.print("Hi I'm Titanoboa, MODULE #: ");
  USB_COM_PORT.println(myModuleNumber, DEC);

  for(int i=4;i<=7;i++)
  {
    pinMode(i,OUTPUT);
    analogWrite(i,0);
  }

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
  int actuator;
  
  // check for command from USB serial to enter manual control
  // NOTE: this is a quick hack so that we can adjust actuators quickly if they get out of wack
  //       this only puts one module in manual control, never operate titanoboa like this!!!
  //       in future manual control mode should be selected from the joystick and all modules enter 
  //       manual control mode and return to normal operation together
  if (USB_COM_PORT.available()>0)
  {
    USB_COM_PORT.print("checking for command on USB\n");
    if (USB_COM_PORT.read() == 'M')
    {
      //confirm this wasn't random noise
      if (USB_COM_PORT.read() == 'M')
      {
        manualControl();
      }
    }
  }

  if(INPUT_SERIAL.available())
  {
    message = INPUT_SERIAL.read();
    USB_COM_PORT.print("received message from xbee: ");
    USB_COM_PORT.println(message);

    if (message == 's')
    {
      currentTime = millis();
      timeDifference = currentTime - lastAngleReceivedTime;

      // setpoints to come, wait for all data to arrive, data includes angle and propagation delay
      while(INPUT_SERIAL.available() < 3);
      // get angle
      message = INPUT_SERIAL.read();
      // throttle delay low byte comes first followed by high byte
      throttleDelay = (INPUT_SERIAL.read());
      throttleDelay += (INPUT_SERIAL.read() << 8);

      // throttle delay of 0 means this is the first angle
      if ((timeDifference > (throttleDelay - 20)) && (timeDifference < (throttleDelay + 20)) || throttleDelay == 0)
      {
        // Delay between receiving angles is within tolerance ok to send the modules
        TAIL_SERIAL.write('s');
        TAIL_SERIAL.write(message);
      }
      else
      {
        USB_COM_PORT.print("droped\n\n");
      }

      USB_COM_PORT.print("time betwen: ");
      USB_COM_PORT.print(timeDifference, DEC);
      USB_COM_PORT.print("\tthrottleDelay: ");
      USB_COM_PORT.println(throttleDelay, DEC);

      // update last time an angle was received
      lastAngleReceivedTime = currentTime;
      return;
    }

    if(message == 'h')
    {
      while(INPUT_SERIAL.available() < 1)
      {
        delay(1);
      }
      TAIL_SERIAL.write(message);

      message = INPUT_SERIAL.read();
      USB_COM_PORT.print("received message from xbee: ");
      USB_COM_PORT.println(message);
      
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
      return;
    }

    TAIL_SERIAL.write(message);
  }
}//end loop()


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
  
  USB_COM_PORT.print("\nManual Control mode entered\n");
  USB_COM_PORT.print("commands: j or h to select jaw or head actuator\n");
  USB_COM_PORT.print("          k/l - actuation\n");
  USB_COM_PORT.print("          d'v' - adjust actuation delay, where v=s(small),m(medium),l(large)\n");
  USB_COM_PORT.print("          s - stop motor\n");
  USB_COM_PORT.print("          q - quit\n");
  
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      byteIn = USB_COM_PORT.read();
      
      switch(byteIn)
      {
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
            analogWrite(JAW_CLOSE, 255);
          }
          else if (actuatorSel == 1)
          {
            analogWrite(HEAD_LOWER, 255);
          }
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
            analogWrite(JAW_OPEN, 255);
          }
          else if (actuatorSel == 1)
          {
            analogWrite(HEAD_RAISE, 255);
          }
          delay(400);
          StopMov();
          USB_COM_PORT.print("done, motor can turn off\n");
          break;
        
        case 's':
          StopMov();
          USB_COM_PORT.print("STOPPED\n");
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
  StopMov(): In manual mode, stops movement of all actuators.
 *************************************************************************************/
void StopMov()
{
  analogWrite(JAW_OPEN, 0);
  analogWrite(JAW_CLOSE, 0);
  analogWrite(HEAD_RAISE, 0);
  analogWrite(HEAD_LOWER, 0);

  return;
}

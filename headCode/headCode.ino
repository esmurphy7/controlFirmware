/*

  headCode.ino - Controls head of Titanoboa
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA to control the entire
  Titanoboa snake. It is the brain of the operation and it schdeules all
  communication. 

  Task list:
  - Requests status of switches and knobs from the joystick
  - Sets new settings and setpoints to every module
  - Tells every modules to execute PID loops on their actuators
  - Requests diagnostic data and sets it out over wifi.
*/

#include "titanoboa_headboard_pins.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "EEPROM.h"

#define INPUT_SERIAL Serial3            // Xbee
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging

// Enable serial stream writing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

// Variables for controlling titanoboa. Currently this data comes
// from the joystick, but in the future it could come from an 
// Android tablet or a PC.
class ControllerData 
{
  public:
  boolean right;
  boolean left;
  boolean up;
  boolean down;
  boolean killSwitchPressed;
  boolean openJaw;
  boolean closeJaw;
  boolean straightenVertical;
  boolean straightenVertOnTheFly;
  boolean calibrate;
  byte motorSpeed;
  unsigned short propagationDelay;
  unsigned short spareKnob2;
  unsigned short spareKnob4;
} controller;

// Ethernet/Wi-Fi variables
EthernetUDP Udp;
byte macAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
byte localIP[] = { 192, 168, 1, 2 };
byte broadcastIP[] = { 192, 168, 1, 255 };
unsigned int broadcastPort = 12345;
unsigned int localPort = 12345;

// Setpoints for all 30 vertibrae in the snake
byte vertSetpoints[30];
byte horzSetpoints[30];
boolean lights[30];

// Other data
byte numberOfModules = 0;
boolean joystickIsConnected = false;

/*************************************************************************
 setup(): Initializes serial ports, led and actuator pins
**************************************************************************/
void setup()
{
  // Initialize serial ports
  USB_COM_PORT.begin(115200);
  TAIL_SERIAL.begin(115200);
  INPUT_SERIAL.begin(115200);
  
  // Initialize Ethernet Board
  Ethernet.begin(macAddress, localIP);
  Udp.begin(localPort);  
  
  // Load saved settings
  numberOfModules = EEPROM.read(0);
  USB_COM_PORT.println("Hi I'm the Titanoboa Head!");  
  USB_COM_PORT.print("Number of modules: ");
  USB_COM_PORT.println(numberOfModules);

  // Initialize all the vertibrae setpoints
  for (int i = 0; i < 30; ++i)
  {
    horzSetpoints[i] = '3';
    vertSetpoints[i] = '3';    
    lights[i] = false;    
  }
  
  // LED Outputs
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);  

  // We have the ability to do actuator dithering, but it's not 
  // really that useful the way our electronics are setup
  pinMode(DITHER, OUTPUT);
  digitalWrite(DITHER, LOW);
  
  // Initialize actuator PWM outputs
  for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATOR[i],OUTPUT);
    analogWrite(HEAD_ACTUATOR[i],0);
  }
  
  // Initialize actuator control outputs
  // Setting control outputs to zero selects odd pairs (1,3,5 as opposed to 2,4,6)  
  for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATOR_CTRL[i], OUTPUT);
    digitalWrite(HEAD_ACTUATOR_CTRL[i], LOW);
  }

  // Initialize position sensor inputs.
  for(int i=0;i<5;i++)
  {
    pinMode(HEAD_POS_SENSOR[i],INPUT);
  }
}

/*************************************************************************
 loop(): Titanoboa's main thought process. It tells the whole snake 
         what to do!
**************************************************************************/
void loop()
{
  unsigned long loopStartTime = millis();
  
  // Check USB serial for command to enter manaul mode  
  if (USB_COM_PORT.available() > 0)
  {
    if (USB_COM_PORT.read() == 'M')
    {
      manualControl(); 
    }    
  }
  
  // Get the status of each knob and switch on the joystick
  readAndRequestJoystickData();
  
  // Send new settings to each module
  updateSetpoints();  
  sendSetpointsAndSettings();

  // Tell the modules to run their PID loops
  runPIDLoops();
  
  // Send diagnostic data
  getAndSendDiagnostics();
  
  // If the calibrate button is pressed
  
  // TODO: This is a hack to ignore a calibrate command immediately
  // following a calirate. Find a better way to fix this.
  static boolean calibrateButtonMemory = false;
  
  if (controller.calibrate && 
      calibrateButtonMemory == false && 
      controller.killSwitchPressed)
  {
    calibrateButtonMemory = true;
    
    // Find out how many modules are connected
    countNumberOfModules();
   
    // Calibrate all sensors    
    runSensorCalibration();
  }
  else
  {
    calibrateButtonMemory = false;
  }
  
  int loopTime = millis() - loopStartTime;
  if (loopTime < 50)
  {
    delay(50 - loopTime);    
  }
}

/**************************************************************************************
  getAndSendDiagnostics(): Gets information from the modules and sends it over WiFi
 *************************************************************************************/
void getAndSendDiagnostics()
{
  // Clear the packet data array
  static byte packet[125];
  for (int i = 0; i < 125; ++i)
  {
    packet[i] = 0;
  }
  
  // What type of diagnostics packet should we send?
  TAIL_SERIAL.write('d');
  
  if (getHeadAndModuleDiagnostics(packet) == false)
  {
    return; 
  }
  
  // Send the diagnostics packet
  Udp.beginPacket(broadcastIP, broadcastPort);  
  for (int i = 0; i < 125; ++i)
  {
    Udp.write(packet[i]);
  }
  Udp.endPacket();
}

/**************************************************************************************
  getHeadAndModuleDiagnostics(): Fills the provided buffer with head and module information.
                                 Note: The buffer must be at least 125 bytes!!
 *************************************************************************************/
boolean getHeadAndModuleDiagnostics(byte* packet)
{
  // Fill in head information
  int headBattery = map(analogRead(BAT_LEVEL_24V),0,1023,0,25000);
  packet[0] = 1;
  packet[1] = numberOfModules;
  packet[2] = highByte(headBattery);
  packet[3] = lowByte(headBattery);

  // Tell modules the packet type
  TAIL_SERIAL.write(1);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char moduleData[6];  
    if (TAIL_SERIAL.readBytes(moduleData, 6) < 6)
    {
      USB_COM_PORT << "ERROR: Did not recieve full packet from module " << i + 1 << "\n";
      return false;
    }
    packet[i * 6 + 0 + 4] = moduleData[0];
    packet[i * 6 + 1 + 4] = moduleData[1];
    packet[i * 6 + 2 + 4] = moduleData[2];
    packet[i * 6 + 3 + 4] = moduleData[3];
    packet[i * 6 + 4 + 4] = moduleData[4];
    packet[i * 6 + 5 + 4] = moduleData[5];
  }
  return true;
}

/**************************************************************************************
  waitForModuleAcknowledgments(): Waits for each module to report back it's module number.
 *************************************************************************************/
void waitForModuleAcknowledgments(char* commandName, short timeout)
{
  // Only wait so long for acknowledgments
  TAIL_SERIAL.setTimeout(timeout);
  
  // Wait for acknowledgments
  char acks[6];  
  byte acksRecieved = TAIL_SERIAL.readBytes(acks, numberOfModules);
  if (acksRecieved != numberOfModules)
  {
    USB_COM_PORT << "ERROR: Only " << acksRecieved << " of " << numberOfModules << 
      " modules acknowledged " << commandName << " command within " << timeout << "ms\n";
    return;
  }
  
  // Each module should acknowledge by sending us it's module number
  // Make sure this is what happened. 
  for (int i = 0; i < numberOfModules; ++i)
  {
    if (acks[i] != i + 1)
    {
      USB_COM_PORT << "ERROR: Recieved bad acknowledgement from module # " << i + 1 << "\n";
    }
  }
}

/**************************************************************************************
  runSensorCalibration(): Calibrates the vertical and horizontal sensors on the actuators
 *************************************************************************************/
void runSensorCalibration()
{
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write('c');
  waitForModuleAcknowledgments("calibrate", 15000); 
  
  // Deinitialize all actuator setpoints. The snake is straight and will
  // need to start slithering from scratch.
  for (int i = 0; i < 30; ++i)
  {
    horzSetpoints[i] = '3';
    vertSetpoints[i] = '3';    
  }
}

/**************************************************************************************
  runPIDLoops(): Tells each module to execute their PID loops
 *************************************************************************************/
void runPIDLoops()
{
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write('p');
  waitForModuleAcknowledgments("pid", 40);
}

/**************************************************************************************
  countNumberOfModules(): Counts the number of modules connected to the serial daisy chain.
 *************************************************************************************/
void countNumberOfModules()
{
  USB_COM_PORT.print("Counting modules... ");
  TAIL_SERIAL.write('n');
  
  // Tell the first module it's module number 1
  TAIL_SERIAL.write(1);
  delay(500);
  
  // Record number of modules that responded
  numberOfModules = TAIL_SERIAL.available();
  EEPROM.write(0, numberOfModules);  
  
  clearSerialBuffer(TAIL_SERIAL);  
  USB_COM_PORT.print("Found ");
  USB_COM_PORT.println(numberOfModules);
}

/**************************************************************************************
  sendSetpointsAndSettings(): Sends the settings down to every module.
 *************************************************************************************/
void sendSetpointsAndSettings()
{
  clearSerialBuffer(TAIL_SERIAL);
  byte settings[125];
  
  // Copy in setpoints
  for (int i = 0; i < 30; ++i)
  {
    settings[i] = horzSetpoints[i];
    settings[i + 30] = vertSetpoints[i];
  }
  
  // Copy in lights
  for (int i = 0; i < 5; ++i)
  {
    byte moduleLights = 0;
    moduleLights += (byte)lights[i * 5 + 0] & B00000001;
    moduleLights += (byte)lights[i * 5 + 1] & B00000010;
    moduleLights += (byte)lights[i * 5 + 2] & B00000100;
    moduleLights += (byte)lights[i * 5 + 3] & B00001000;
    moduleLights += (byte)lights[i * 5 + 4] & B00010000;
    settings[60 + i] = moduleLights;
  }
  
  // Misc 
  settings[70] = 0;
  settings[70] += controller.killSwitchPressed & B00000001;
  settings[72] = controller.motorSpeed;
  
  // Send settings
  TAIL_SERIAL.write('s');
  TAIL_SERIAL.write(settings, 125);
  
  // Wait for ack
  waitForModuleAcknowledgments("settings", 40);
}

/**************************************************************************************
  updateSetpoints(): Propagates the setpoints when it it time to do so.
 *************************************************************************************/
void updateSetpoints()
{
  static unsigned long lastUpdateTime = 0;

  // Disable vertical actuation. If (1) the straighten vertical switch is
  // pressed or (2) vertical straighten on the fly is enabled, set all
  // verticals to straight.
  byte allVertSetpoints = '3';
  if (controller.killSwitchPressed &&
      (controller.straightenVertOnTheFly || controller.straightenVertical))
  {
      allVertSetpoints = '2';
  }
  for (int i = 0; i < 30; ++i)
  {
    vertSetpoints[i] = allVertSetpoints;
  }

  // Propagate horizontal setpoints if it's time to do so.
  if (controller.killSwitchPressed &&
      (controller.left || controller.right) &&
      (millis() - lastUpdateTime > controller.propagationDelay))
  {
    for (int i = 29; i > 0; --i)
    {
      horzSetpoints[i] = horzSetpoints[i - 1];
    }
    
    // The first actuator gets the new setpoint
    if (controller.right)
    {
      horzSetpoints[0] = '1';
    }
    if (controller.left)
    {
      horzSetpoints[0] = '0';
    }
    lastUpdateTime = millis();  
  }
}

/**************************************************************************************
  readAndRequestJoystickData(): Asks the joystick for data and reads its back. If the joystick hasn't
                     responded for 300ms it retries. After 1 retry we are disconnected.
                     It takes between 39 to 210ms for the joystick to responsed (avg: 76ms)
 *************************************************************************************/
void readAndRequestJoystickData()
{
  static unsigned long lastJoystickRequestTime = 0;
  static byte joystickRetryAttempts = 0;  
  
  // Check if there's a full joystick packet available
  if (INPUT_SERIAL.available() < 30)
  {
    // There's no joystick data. If it's been 300ms since 
    // the last request. Retry: Ask the joystick for data again.
    if (millis() - lastJoystickRequestTime > 300)
    { 
      // If we've retried already, now we know the joystick is disconnected!
      if (joystickRetryAttempts > 0)
      {
        USB_COM_PORT.println("ERROR: No joystick data. It is Disconnected.");
        joystickIsConnected = false;
        controller.killSwitchPressed = false;
      }
      ++joystickRetryAttempts;
      clearSerialBuffer(INPUT_SERIAL);
      INPUT_SERIAL.write('j');
      lastJoystickRequestTime = millis();       
    }
    return;
  }
 
  // Success! We have new data, so we are connected to the joystick.
  joystickIsConnected = true; 
  joystickRetryAttempts = 0;
  
  // Sort our new data.
  char packet[30];
  INPUT_SERIAL.readBytes(packet, 30);
  
  controller.killSwitchPressed = (boolean)packet[0];
  controller.left = (boolean)packet[1];
  controller.right = (boolean)packet[2];
  controller.up = (boolean)packet[3];
  controller.down = (boolean)packet[4];
  controller.openJaw = (boolean)packet[5];
  controller.closeJaw = (boolean)packet[6];
  controller.straightenVertical = (boolean)packet[7];
  controller.calibrate = (boolean)packet[8];
  controller.straightenVertOnTheFly = (boolean)packet[9];
  controller.motorSpeed = packet[12];
  controller.propagationDelay = word(packet[13], packet[14]);
  controller.spareKnob2 = word(packet[15], packet[16]);
  controller.spareKnob4 = word(packet[17], packet[18]);

  // Request another joystick packet  
  lastJoystickRequestTime = millis();
  INPUT_SERIAL.write('j');  

  /*USB_COM_PORT.println(controller.killSwitchPressed);
  USB_COM_PORT.println(controller.left);
  USB_COM_PORT.println(controller.right);
  USB_COM_PORT.println(controller.up);
  USB_COM_PORT.println(controller.down);
  USB_COM_PORT.println(controller.openJaw);
  USB_COM_PORT.println(controller.closeJaw);
  USB_COM_PORT.println(controller.straightenVertical);
  USB_COM_PORT.println(controller.calibrate);
  USB_COM_PORT.println(controller.straightenVertOnTheFly);
  USB_COM_PORT.println(controller.motorSpeed);
  USB_COM_PORT.println(controller.propagationDelay);
  USB_COM_PORT.println(controller.spareKnob2);
  USB_COM_PORT.println(controller.spareKnob4);*/
  
  return;
}

/**************************************************************************************
  clearSerialBuffer(): In Arduino 1.0, Serial.flush() no longer does what we want!
                       http://arduino.cc/en/Serial/Flush
 *************************************************************************************/
void clearSerialBuffer(HardwareSerial &serial)
{
  while (serial.available() > 0)
  {
    serial.read();
  }
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
  
  TAIL_SERIAL.write("h");
  
  headjawmessage = INPUT_SERIAL.read();
  USB_COM_PORT.print("received jaw message: ");
  USB_COM_PORT.println(headjawmessage);
  
  if(headjawmessage == '0')
  {
    actuator = -1;
    digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);
      
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
    digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);

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
    digitalWrite(HEAD_CTRL, HEAD_RAISE_CTRL_SELECT);
  }
  else if(headjawmessage == '3')
  {
    actuator = HEAD_LOWER;
    digitalWrite(HEAD_CTRL, HEAD_LOWER_CTRL_SELECT);
  }
  else if(headjawmessage == '4')
  {
    actuator = AUX_EXTEND;
    digitalWrite(AUX_CTRL, AUX_EXTEND_CTRL_SELECT);
  }
    else if(headjawmessage == '5')
  {
    actuator = AUX_RETRACT;
    digitalWrite(AUX_CTRL, AUX_RETRACT_CTRL_SELECT);
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
  }
  INPUT_SERIAL.flush(); //This may not be the smartest action we can take
  return; 
}

/**************************************************************************************
  manualControl(): Allows for manual actuator control over the USB_COM_PORT
 *************************************************************************************/
void manualControl()
{
  boolean manual = true;
  char byteIn = 'z';
  int actuatorSel = 0;
  int actuationDelay = 200;
  char delaySetting = 'm';
  int led_choice = -1;
  int sensors[6];
  
  displayMenu();
  
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      byteIn = USB_COM_PORT.read();
      
      // characters in use: a, b, c, j, h, u, m, d, l, k, s, q
      switch(byteIn)
      {
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
        case 'u':
          actuatorSel = 2;
          StopMov();
          USB_COM_PORT.print("Auxiliary actuator selected\n");
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
            digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);
            analogWrite(JAW_CLOSE, 255);
          }
          else if (actuatorSel == 1)
          {
            digitalWrite(HEAD_CTRL, HEAD_LOWER_CTRL_SELECT);
            analogWrite(HEAD_LOWER, 255);
          }
          else if(actuatorSel == 2)
          {
            digitalWrite(AUX_CTRL, AUX_EXTEND_CTRL_SELECT);
            analogWrite(AUX_EXTEND, 255);
          }
          delay(actuationDelay);
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
            digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);
            analogWrite(JAW_OPEN, 255);
          }
          else if (actuatorSel == 1)
          {
            digitalWrite(HEAD_CTRL, HEAD_RAISE_CTRL_SELECT);
            analogWrite(HEAD_RAISE, 255);
          }
          else if(actuatorSel == 2)
          {
            digitalWrite(AUX_CTRL, AUX_RETRACT_CTRL_SELECT);
            analogWrite(AUX_RETRACT, 255);
          }
          delay(actuationDelay);
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
          INPUT_SERIAL.flush();
          TAIL_SERIAL.flush();
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
    USB_COM_PORT.print("commands: j or h to select jaw or head actuator.\n");
    USB_COM_PORT.print("          u to select Aux actuator\n");
    USB_COM_PORT.print("\n");
    USB_COM_PORT.print("          a/b - leds [0-6]\n");
    USB_COM_PORT.print("          c - list sensor values\n");
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
  analogWrite(AUX_EXTEND, 0);
  analogWrite(AUX_RETRACT, 0);
  return;
}


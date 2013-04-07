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

// Write to the USB com port only if we are in debug mode
boolean inDebugMode = false;
#define DEBUG_STREAM if(inDebugMode) USB_COM_PORT 

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
  unsigned short ledDelay; // light delay
  unsigned short sendHeartBeatDelay;
} controller;

// Ethernet/Wi-Fi variables
EthernetUDP Udp;
byte macAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
byte localIP[] = { 192, 168, 1, 2 };
byte broadcastIP[] = { 192, 168, 1, 255 };
unsigned int broadcastPort = 12345;
unsigned int localPort = 12345;

// Setpoints for all 30 vertibrae in the snake
// Angles are from 0 to 254, 255 means uninitialized.
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

  // Initialize setponts
  //   - Actuators are disabled.
  //   - Lights are off
  for (int i = 0; i < 30; ++i)
  {
  
    horzSetpoints[i] = 255;
    vertSetpoints[i] = 255;    
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
    pinMode(HEAD_ACTUATORS[i],OUTPUT);
    analogWrite(HEAD_ACTUATORS[i],0);
  }
  
  // Initialize actuator control outputs
  // Setting control outputs to zero selects odd pairs (1,3,5 as opposed to 2,4,6)  
  for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATOR_CTRLS[i], OUTPUT);
    digitalWrite(HEAD_ACTUATOR_CTRLS[i], LOW);
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
  ////////////////////////////////////////////////////////////////////
  // This block of code runs the core functionality of Titanoboa 
  
  readAndRequestJoystickData();
  updateLights();
  updateSetpoints();  
  sendSetpointsAndSettings();
  runPIDLoops();
  getAndSendDiagnostics();
  runPIDLoops();
  
  ////////////////////////////////////////////////////////////////////
  // The following functions don't run on a regular basis.
  // When they do run, they take a long time.
  
  // Should we calibrate?
  if (controller.calibrate && controller.killSwitchPressed)
  {
    countNumberOfModules();
    runVertSensorCalibration();
    synchronizeWithJoystick();
    return;
  }
  
  // Should we open the jaw?
  if (controller.openJaw && controller.killSwitchPressed)
  {
    openTheJaw();
    synchronizeWithJoystick();  
    return;
  }

  // Should we close the jaw?
  if (controller.closeJaw && controller.killSwitchPressed)
  {
    closeTheJaw();
    synchronizeWithJoystick(); 
    return;
  }
  
  // Should we enter manual mode?
  if (USB_COM_PORT.available() > 0 && USB_COM_PORT.read() == 'M')
  {
    manualControl();
    synchronizeWithJoystick();
    return;
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
  // Cycle through each of the 5 packet each time this function is called.
  static byte packetType = 0;
  ++packetType;
  if (packetType > 5)
  {
    packetType = 1;
  }

  // Fill the packet with data
  boolean retVal = false;
  switch (packetType)
  {
    case 1:
      retVal = getHeadAndModuleDiagnostics(packet);
      break;
    case 2:
      retVal = getHorzAngleDiagnostics(packet);
      break;
    case 3:
      retVal = getVertAngleDiagnostics(packet);
      break;
    case 4:
      retVal = getHorzCalibrationDiagnostics(packet);
      break;
    case 5:
      retVal = getVertCalibrationDiagnostics(packet);
      break;
    default: 
      DEBUG_STREAM << "ERROR: Invalid packet type = " << packetType << "\n";
      return;
  }
  if (retVal == false)
  {
    // Packet was not filled for some reason.
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
  getHeadAndModuleDiagnostics(): Fills a 125 byte buffer with head and general module info
 *************************************************************************************/
boolean getHeadAndModuleDiagnostics(byte* buffer)
{ 
  buffer[0] = 1;
  
  // Fill in head information
  int headBattery = map(analogRead(BAT_LEVEL_24V),0,1023,0,25000);
  buffer[1] = numberOfModules;
  buffer[2] = highByte(headBattery);
  buffer[3] = lowByte(headBattery);

  // Tell modules to send us this type of diagnostics
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(1);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[6];  
    if (TAIL_SERIAL.readBytes(data, 6) < 6)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all Module info from module " << i + 1 << "\n";
      return false;
    }
    buffer[i * 6 + 0 + 4] = data[0];
    buffer[i * 6 + 1 + 4] = data[1];
    buffer[i * 6 + 2 + 4] = data[2];
    buffer[i * 6 + 3 + 4] = data[3];
    buffer[i * 6 + 4 + 4] = data[4];
    buffer[i * 6 + 5 + 4] = data[5];
  }
  return true;
}

/**************************************************************************************
  getHorzAngleDiagnostics(): Fills a 125 byte buffer with horizontal angle info
 *************************************************************************************/
boolean getHorzAngleDiagnostics(byte* buffer)
{
  buffer[0] = 2;
  
  // Tell modules to send us this type of diagnostics
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(2);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all HorzAngle info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
  }
  return true;
}

/**************************************************************************************
  getVertAngleDiagnostics(): Fills a 125 byte buffer with vertical angle info
 *************************************************************************************/
boolean getVertAngleDiagnostics(byte* buffer)
{
  // Tell the wifi listeners what type of packet this is
  buffer[0] = 3;
  
  // Tell modules to send us this type of diagnostics
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(3);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all VertAngle info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
  }
  return true;
}

/**************************************************************************************
  getHorzCalibrationDiagnostics(): Fills a 125 byte buffer with horizontal calibration info
 *************************************************************************************/
boolean getHorzCalibrationDiagnostics(byte* buffer)
{
  // Tell the wifi listeners what type of packet this is
  buffer[0] = 4;
  
  // Tell modules to send us this type of diagnostics
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(4);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all HorzCalibration info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
  }
  return true;
}

/**************************************************************************************
  getVertCalibrationDiagnostics(): Fills a 125 byte buffer with vertical calibration info
 *************************************************************************************/
boolean getVertCalibrationDiagnostics(byte* buffer)
{
  // Tell the wifi listeners what type of packet this is
  buffer[0] = 5;
  
  // Tell modules to send us this type of diagnostics
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(5);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all VertCalibration info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
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
    DEBUG_STREAM << "ERROR: Only " << acksRecieved << " of " << numberOfModules << 
      " modules acknowledged " << commandName << " command within " << timeout << "ms\n";
    return;
  }
  
  // Each module should acknowledge by sending us it's module number
  // Make sure this is what happened. 
  for (int i = 0; i < numberOfModules; ++i)
  {
    byte data = acks[i];
    if (data != i + 1)
    {
      DEBUG_STREAM << "ERROR: Recieved bad acknowledgement from module # " << i + 1 << " (" 
        << data << ")\n";
    }
  }
  clearSerialBuffer(TAIL_SERIAL);
}

/**************************************************************************************
  runHorzSensorCalibration(): Calibrates the horizontal sensors on the actuators
 *************************************************************************************/
void runHorzSensorCalibration()
{
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.print("ch");
  waitForModuleAcknowledgments("calibrate horz", 15000); 
  
  // The snake will end up straight.
  for (int i = 0; i < 30; ++i)
  {
    horzSetpoints[i] = 127;  
  }
}

/**************************************************************************************
  runHorzSensorCalibration(): Calibrates the horizontal sensors on the actuators
 *************************************************************************************/
void runVertSensorCalibration()
{
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write("cv");
  waitForModuleAcknowledgments("calibrate vert", 15000); 
  
  // The snake will end up straight.
  for (int i = 0; i < 30; ++i)
  {
    vertSetpoints[i] = 127;  
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
    moduleLights += ((byte)lights[i * 5 + 0]) << 0;
    moduleLights += ((byte)lights[i * 5 + 1]) << 1;
    moduleLights += ((byte)lights[i * 5 + 2]) << 2;
    moduleLights += ((byte)lights[i * 5 + 3]) << 3;
    moduleLights += ((byte)lights[i * 5 + 4]) << 4;
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
  byte allVertSetpoints = 255;
  if (controller.killSwitchPressed &&
      (controller.straightenVertOnTheFly || controller.straightenVertical))
  {
      allVertSetpoints = 127;
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
      horzSetpoints[0] = 245;
    }
    if (controller.left)
    {
      horzSetpoints[0] = 10;
    }
    lastUpdateTime = millis();  
  }
}

/**************************************************************************************
  updateLights(): Controls all the lights of the snake.
 *************************************************************************************/
 
void updateLights()
{
  static unsigned long lastUpdateTime = 0;
  
  // Propagate random pulses down the snake when we're slithering..
  // .. like nervous signals though a nervous system.
  if (controller.killSwitchPressed && 
      (millis() - lastUpdateTime > controller.ledDelay))
  {
    for (int i = 29; i > 0; --i)
    {
      lights[i] = lights[i - 1];
    }
    lights[0] = (boolean)random(0, 2);
    lastUpdateTime = millis();
  }
  
  // Heart beats while we're stationary
  if (!controller.killSwitchPressed)
  {
    static boolean firstBeatSent = false;    

    // Propegate all lights
    for (int i = 29; i > 0; --i)
    {
      lights[i] = lights[i - 1];
    }
    
    // Default to propagating off light
    lights[0] = false;
    
    // Propagate heat beats when its time
    if (!firstBeatSent && (millis() - lastUpdateTime > (controller.sendHeartBeatDelay * 4)))
    {
      lights[0] = true;
      firstBeatSent = true;
      lastUpdateTime = millis();
    }
    else if (firstBeatSent && (millis() - lastUpdateTime > controller.ledDelay))
    {
      lights[0] = true;
      firstBeatSent = false;
      lastUpdateTime = millis();
    }
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
        DEBUG_STREAM << "ERROR: No joystick data. It is Disconnected.";
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
  controller.ledDelay = word(packet[15], packet[16]);
  controller.sendHeartBeatDelay = word(packet[17], packet[18]);

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
  USB_COM_PORT.println(controller.ledDelay);
  USB_COM_PORT.println(controller.sendHeartBeatDelay);*/
  
  return;
}

/**************************************************************************************
  synchronizeWithJoystick(): Clears away any previous data from the joystick and makes sure
                             a fresh set is available. It's important to do this if we just
                             executed a command like calibrate where the joystick data wasn't
                             updating.
 *************************************************************************************/
void synchronizeWithJoystick()
{
  USB_COM_PORT.print("Getting back in sync with joystick. Waiting for data... ");
  do
  {
    clearSerialBuffer(INPUT_SERIAL);
    INPUT_SERIAL.write('j');
    delay(500);
  } 
  while (INPUT_SERIAL.available() != 30);
  USB_COM_PORT.println("Synchronized!");
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
  openTheJaw(): Opens the jaw.
 *************************************************************************************/
void openTheJaw()
{
  USB_COM_PORT << "Opening this jaw... ";
  
  // Tell the first module to turn on the motor for a bit
  TAIL_SERIAL.write("h");
  
  // Slowly open and close to jaw opening valve
  digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);  
  for(int i=100; i<=255; i++)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(10);
  }
  for(int i=255; i>=50; i--)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(6);
  }
  analogWrite(JAW_ACTUATOR, 0);
  
  USB_COM_PORT << "Done\n";   
}

/**************************************************************************************
  closeTheJaw(): Closes the jaw.
 *************************************************************************************/
void closeTheJaw()
{
  USB_COM_PORT << "Closing this jaw... ";
  
  // Tell the first module to turn on the motor for a bit
  TAIL_SERIAL.write("h");
  
  // Slowly open and close to jaw closing valve
  digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);
  for(int i=0; i<=255; i++)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(4);
  }
  for(int i=255; i>=0; i--)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(4);
  }
  analogWrite(JAW_ACTUATOR, 0);
  
  USB_COM_PORT << "Done\n";  
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
  byte numberOfModulesNew;
  
  displayMenu();
  
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      byteIn = USB_COM_PORT.read();
      
      // characters in use: a, b, c, j, h, u, m, d, l, k, n, t, s, q, y, u, i
      switch(byteIn)
      {
        case 'y':
          USB_COM_PORT << "\nCalibrating horizontals... ";
          runHorzSensorCalibration();
          USB_COM_PORT << "Done\n";
          break;
          
        case 'i':
          USB_COM_PORT << "\nCalibrating verticals... ";
          runVertSensorCalibration();
          USB_COM_PORT << "Done\n";
          break;
          
        case 'm':
          countNumberOfModules();
          break;
          
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
            analogWrite(JAW_ACTUATOR, 255);
          }
          else if (actuatorSel == 1)
          {
            digitalWrite(HEAD_CTRL, HEAD_LOWER_CTRL_SELECT);
            analogWrite(HEAD_ACTUATOR, 255);
          }
          else if(actuatorSel == 2)
          {
            digitalWrite(AUX_CTRL, AUX_EXTEND_CTRL_SELECT);
            analogWrite(AUX_ACTUATOR, 255);
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
            analogWrite(JAW_ACTUATOR, 255);
          }
          else if (actuatorSel == 1)
          {
            digitalWrite(HEAD_CTRL, HEAD_RAISE_CTRL_SELECT);
            analogWrite(HEAD_ACTUATOR, 255);
          }
          else if(actuatorSel == 2)
          {
            digitalWrite(AUX_CTRL, AUX_RETRACT_CTRL_SELECT);
            analogWrite(AUX_ACTUATOR, 255);
          }
          delay(actuationDelay);
          StopMov();
          USB_COM_PORT.print("done, motor can turn off\n");
          break;
        
        case 'n':
          USB_COM_PORT << "Manually program number of modules (0-9)...\n";
          while(USB_COM_PORT.available() < 1);
          numberOfModulesNew = USB_COM_PORT.read() - 48;
          if (numberOfModulesNew < 0 || numberOfModulesNew > 9)
          {
            USB_COM_PORT << "Invalid\n";
            break;
          }
          USB_COM_PORT << "Now set to " << numberOfModulesNew << "\n";
          EEPROM.write(0, numberOfModulesNew);
          numberOfModules = numberOfModulesNew;
          break;

        case 's':
          StopMov();
          USB_COM_PORT.print("STOPPED\n");
          break;
          
       case 'p':
          if (inDebugMode)
          {
            USB_COM_PORT.println("Debug stream messages OFF");
            inDebugMode = false;
          }
          else
          {
            USB_COM_PORT.println("Debug stream messages ON");
            inDebugMode = true;
          }
          break;
        case 't':
          runCommunicationTest();
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
  runCommunicationTest(): Makes sure all modules are responding and the last module is 
                          terminated. Returns true if it passed the test.
 *************************************************************************************/
boolean runCommunicationTest()
{
  clearSerialBuffer(TAIL_SERIAL);
  boolean allError = false;
  
  for (int i = 1; i <= 20; ++i)
  {
    USB_COM_PORT << "Running test " << i << " of 20.\n";
    TAIL_SERIAL.write('t');
    delay(100);
    
    // We should recieve modules number (1 2 3 4) followed by ASCII 't' aka ASCII 116
    long startTime = millis();
    int num = 1;
    boolean gotLastModuleT = false;
    boolean error = false;
    
    while(millis() - startTime < 150)
    {
      // Wait for data
      if (!TAIL_SERIAL.available())
      {
        continue;
      }
      byte data = TAIL_SERIAL.read();
      
      // If we've heard from every module look for ASCII charater 't'
      if (num > numberOfModules)
      {
        if (data == 't')
        {
          USB_COM_PORT << "Module #" << num - 1 << " knows it's the last module!\n";
          gotLastModuleT = true;
        }
        else
        {
          USB_COM_PORT << "ERROR: Didn't recieve 't' from last module #" << num -1 << ". Recieved ASCII " << data << " instead.\n";
          error = true;
        }
        break;              
      }
      
      // Look for each module to respond with it's module number (in sequence)
      if (data == num)
      {
        USB_COM_PORT << "Module #" << num << " is talking!\n";
      }
      else
      {
        USB_COM_PORT << "ERROR: Expected Module # '" << num << "'. Recieved '" << data << "'\n";
        error = true;
      }
      ++num;
    }
    // If we didn't recieve data from all modules.
    if (!error && num <= numberOfModules)
    {
      USB_COM_PORT << "ERROR: Did not recieve data from all " << numberOfModules << " modules.\n";
      error = true;     
    }
    // If we didn't recieve 't' from the last module.
    if (!error && !gotLastModuleT)
    {
      USB_COM_PORT << "ERROR: Did not recieve 't' from the last module. Is the serial terminator plugged in?\n";
      error = true;
    }
    USB_COM_PORT << "\n";
    
    allError = (error) ? true : allError;
  }
  
  // Give the result of the tests
  if (allError)
  {
    USB_COM_PORT << "RESULT: ERROR! Some communication tests failed.\n\n";
  }
  else
  {
    USB_COM_PORT << "RESULT: SUCCESS! All communication tests passed.\n\n";
  }
  delay(500);
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
    USB_COM_PORT.print("          n - manually set numberOfModules\n"); 
    USB_COM_PORT.print("          m - auto calibrate numberOfModules\n"); 
    USB_COM_PORT.print("          p - print debug stream\n");     
    USB_COM_PORT.print("          t - test communication\n");     
    USB_COM_PORT.print("          y - calibrate horizontal\n");     
    USB_COM_PORT.print("          i - calibrate vertical\n");     
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n\n");
}


/**************************************************************************************
  StopMov(): In manual mode, stops movement of all actuators.
 *************************************************************************************/
void StopMov()
{
  analogWrite(JAW_ACTUATOR, 0);
  analogWrite(HEAD_ACTUATOR, 0);
  analogWrite(AUX_ACTUATOR, 0);
  return;
}


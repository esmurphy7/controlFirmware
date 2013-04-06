/*

  moduleCode.ino - Controls 1 Titanaboa module of 5 vertebrae  
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA with a BoaShield to
  the control 5 vertical and 5 horizontal actuators of a 
  5 vertebrae module. Titanaboa moves by pushing a sequence of
  angles from head to tail at an interval initiated by the head board.
  
  Vertebrae angles are controlled by a hydrualic system in a PID
  loop with angular position sensors. Communication between the 
  modules is done over a serial daisy chain. 
*/

#include "EEPROM.h"
#include "titanoboa_pins.h"
#include "PIDcontrol.h"

// Defines and constants
#define HEAD_SERIAL Serial1             // Serial to the upstream module
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging

// Enable serial stream writing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

// Memory locations for items stored to EEPROM
#define MY_MODULE_NUMBER_ADDRESS  0
#define HORIZONTAL_CALIBRATION_ADDRESS  2
#define VERTICAL_CALIBRATION_ADDRESS  (HORIZONTAL_CALIBRATION_ADDRESS + 25)
#define VERTICAL_STRAIGHT_ADDRESS  (VERTICAL_CALIBRATION_ADDRESS + 25)

#define CALIBRATE_MOTOR_SPEED  150
#define JAW_MOTOR_SPEED        90

// Static variables
boolean even = false;                   // Unused, will be set in EEPROM after calibration

// Target sensor values for the actuators
// -1 means un initialized
int horzSetpoints[] = {-1,-1,-1,-1,-1};
int vertSetpoints[] = {-1,-1,-1,-1,-1};

// Array of sensor values we recorded as straight for the verticals
int vertStraightArray[] = {-1, -1, -1, -1, -1};

// Timer arrays used to timeout when waiting for an actuator to reach its set point
unsigned long horzTimerArray[] = {0,0,0,0,0};                           
unsigned long vertTimerArray[] = {0,0,0,0,0};  

// Calibration values. The values of the position sensors at the hard limits of actuation.
// Assume a linear sensor characteristic between these two points.
int horzHighCalibration[] = {1023, 1023, 1023, 1023, 1023};
int horzLowCalibration[] = {0, 0, 0, 0, 0};
int vertHighCalibration[] = {1023, 1023, 1023, 1023, 1023};
int vertLowCalibration[]  = {0, 0, 0, 0, 0};

// We configure the deadbands in angle space (0-255). These arrays are a mapping of these
// values to each sensor space. They are calculated by calling calculateDeadbands()
int horzSensorDeadbands[] = {-1,-1,-1,-1,-1};
int vertSensorDeadbands[] = {-1,-1,-1,-1,-1};

byte myModuleNumber = 0;                // My position in the module chain (1,2,3...)
boolean iAmLastModule = false;          // If we are the last module don't read tail serial
byte killSwitchPressed = false;         // If the kill switch isn't pressed. Don't move.
byte motorSpeed = 200;                  // Analog output for pump speed when turned on
byte horzAngleDeadband = 10;            // The horizontal deadband, 0 to 255 range
byte vertAngleDeadband = 20;            // The vertical deadband, 0 to 255 range
int horzActuatorTimeout = 3000;         // Horizontal actuators have this maybe ms to get to setpoint
int vertActuatorTimeout = 1000;         // Vertical actuators have this maybe ms to get to setpoint

// PID Controllers for the horizontal actuators
// Declaration of horizontal PID controllers, default even and odd values applied here
// but get updated during calibration
PIDcontrol PIDcontrollerHorizontal[] = {
  PIDcontrol(HORZ_POS_SENSOR[0], HORZ_ACTUATOR_CTRL[0], HORZ_ACTUATOR[0], even),
  PIDcontrol(HORZ_POS_SENSOR[1], HORZ_ACTUATOR_CTRL[1], HORZ_ACTUATOR[1], !even),
  PIDcontrol(HORZ_POS_SENSOR[2], HORZ_ACTUATOR_CTRL[2], HORZ_ACTUATOR[2], even),
  PIDcontrol(HORZ_POS_SENSOR[3], HORZ_ACTUATOR_CTRL[3], HORZ_ACTUATOR[3], !even),
  PIDcontrol(HORZ_POS_SENSOR[4], HORZ_ACTUATOR_CTRL[4], HORZ_ACTUATOR[4], even),
};

// PID Controllers for the vertical actuators
// Declaration of vertical PID controllers, default even and odd values only apply to
// odd numbered modules
PIDcontrol PIDcontrollerVertical[] = {
  PIDcontrol(VERT_POS_SENSOR[0], VERT_ACTUATOR_CTRL[0], VERT_ACTUATOR[0], !even),
  PIDcontrol(VERT_POS_SENSOR[1], VERT_ACTUATOR_CTRL[1], VERT_ACTUATOR[1], even),
  PIDcontrol(VERT_POS_SENSOR[2], VERT_ACTUATOR_CTRL[2], VERT_ACTUATOR[2], !even),
  PIDcontrol(VERT_POS_SENSOR[3], VERT_ACTUATOR_CTRL[3], VERT_ACTUATOR[3], even),
  PIDcontrol(VERT_POS_SENSOR[4], VERT_ACTUATOR_CTRL[4], VERT_ACTUATOR[4], !even),
};

/****************************************************************************
 setup(): Initializes serial ports, pins and loads EEPROM calibration
**************************************************************************/
void setup()
{
  // Initialize serial ports
  TAIL_SERIAL.begin(115200); 
  HEAD_SERIAL.begin(115200); 
  USB_COM_PORT.begin(115200);

  // Turn off motor pin
  pinMode(MOTOR_CONTROL,OUTPUT);
  analogWrite(MOTOR_CONTROL,0);

  // Initialize solenoid valve control ouputs
  for(int i=0;i<5;i++)
  {
    pinMode(HORZ_ACTUATOR[i],OUTPUT);
    analogWrite(HORZ_ACTUATOR[i],0);
    pinMode(VERT_ACTUATOR[i],OUTPUT);
    analogWrite(VERT_ACTUATOR[i],0);
  }

  // Initialize solenoid valve selection ouputs
  // Each valve has two solenoids: 1 for extending
  // 1 for retracting. This signal selects which one 
  // we are controlling.
  for(int i=0;i<5;i++)
  {
    pinMode(HORZ_ACTUATOR_CTRL[i],OUTPUT);
    pinMode(VERT_ACTUATOR_CTRL[i],OUTPUT);
  }

  // Initialize position sensor inputs.
  for(int i=0;i<5;i++)
  {
    pinMode(HORZ_POS_SENSOR[i],INPUT);
    pinMode(VERT_POS_SENSOR[i],INPUT);
  }

  //initialize LED Pins
  for(int i=0;i<8;i++){
    pinMode(LED[i],OUTPUT);
  }
  
  // Set the constants of the PID controllers
  for(int i=0;i<5;i++)
  {
    PIDcontrollerHorizontal[i].setConstants(50,0,0);
    PIDcontrollerVertical[i].setConstants(50,0,0);
  }

  // Load previous calibration from EEPROM
  myModuleNumber = EEPROM.read(0);
  for(int i=0;i<5;i++)
  { 
    horzHighCalibration[i] = EEPROM.read(i*5+2) + (EEPROM.read(i*5+3) << 8);
    horzLowCalibration[i] = EEPROM.read(i*5+4) + (EEPROM.read(i*5+5) << 8);
    PIDcontrollerHorizontal[i].setEven(EEPROM.read(i*5+6));
  }
  
  // Load vertical straight array from EEPROM
  for(int i=0;i<5;i++)
  {
    vertStraightArray[i] = EEPROM.read(i*5+VERTICAL_STRAIGHT_ADDRESS)
                           + (EEPROM.read(i*5+VERTICAL_STRAIGHT_ADDRESS+1) << 8);
  }

  // setup even/odd for even modules, default values applied at declaration only apply to
  // odd numbered modules
  if (myModuleNumber%2 == 0)
  {
    PIDcontrollerVertical[0].setEven(even);
    PIDcontrollerVertical[1].setEven(!even);
    PIDcontrollerVertical[2].setEven(even);
    PIDcontrollerVertical[3].setEven(!even);
    PIDcontrollerVertical[4].setEven(even);
  }

  // Print out the loaded cabibration and angle array.
  USB_COM_PORT.print("\nHi I'm Titanoboa, MODULE #: ");
  USB_COM_PORT.println(myModuleNumber, DEC);

  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write(200);
  delay(10);
  boolean avail = (TAIL_SERIAL.available() == 1);
  int data = TAIL_SERIAL.read();
  USB_COM_PORT.println(data);
  if (avail && (data == 200))
  {
    iAmLastModule = true;
    USB_COM_PORT.println("... and I'm the last module!");
  }
  
  USB_COM_PORT.print("\nCurrent voltage reading: ");
  USB_COM_PORT.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,25000));

  USB_COM_PORT.println("\n> Loaded Calibration and Initialized Horizontal Angle Array");
  USB_COM_PORT.print("HIGH:     \t");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(horzHighCalibration[i]);
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.print("\nLOW:     \t");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(horzLowCalibration[i]);
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.print("\nEVEN?:  \t"); 
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(PIDcontrollerHorizontal[i].getEven() ? 'T' : 'F');
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.print("\nV_STRAIGHT:\t");  
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(vertStraightArray[i]);
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.println('\n');

  delay(1000);
  USB_COM_PORT.println("\n");
  clearSerialBuffer(HEAD_SERIAL);
  clearSerialBuffer(TAIL_SERIAL);
}


/***********************************************************************************
  loop(): Checks for Upstream Serial or USB Serial commands. Executes them.
 ***********************************************************************************/
 
void loop()
{ 
  // Check the USB port for command to enter manual mode
  if (USB_COM_PORT.available()>0)
  {
    if (USB_COM_PORT.read() == 'M')
    {
      manualControl();
    }
  }

  // Check upstream Serial for commands from the head
  if (HEAD_SERIAL.available() > 0)
  {
    //letters in use: p, s, c, h, g, n, v, t
    char command = HEAD_SERIAL.read();
    switch (command)
    {
      case 'p':
        processRunPIDCommand();
        acknowledgeCommand();
        break;
      case 's':
        processNewSettingsAndSetpoints();
        acknowledgeCommand();
        break;
      case 'n':
        processCountModulesCommand();
        break;
      case 'd':
        processDiagnosticsCommand();
        break;
      case 'c':
        processHorzCalibrateCommand();
        acknowledgeCommand();
        break;
      case 'v':
        processVertCalibrateCommand();
        acknowledgeCommand();
        break;
      case 'h':
        processLongMotorPulseCommand();
        break;
      case 'g':
        processShortMotorPulseCommand();
        break;
      case 't':
        processCommunicationTestCommand();
        break;

      default:
        USB_COM_PORT.print("Invalid command from upstream serial = ");
        USB_COM_PORT.println(command);
        clearSerialBuffer(HEAD_SERIAL);
    }
  }
  
  while (iAmLastModule == false && TAIL_SERIAL.available() > 0)
  {
    byte data = TAIL_SERIAL.read();
    HEAD_SERIAL.write(data);
  }

}//end loop()

/************************************************************************************
  acknowledgeCommand(): Tells the head the command is finished on this module.
                        Only send acknowledgements if the head is programmed to wait
                        for them after that command.
 ***********************************************************************************/
void acknowledgeCommand()
{
  HEAD_SERIAL.write(myModuleNumber);
}

/************************************************************************************
  processCommmunicationTestCommand(): Tests up and down serial communication
 ***********************************************************************************/
void processCommunicationTestCommand()
{
  USB_COM_PORT << "Running communication test. Sending up myModuleNumber = " << myModuleNumber <<"\n";
  
  // Respond with module number so the head knows I got the message.
  HEAD_SERIAL.write(myModuleNumber);
  
  // Pass on test message.
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write('t');
  
  // Check to see if the serial terminator is plugged into my tail serial.
  delay(1);
  if (TAIL_SERIAL.available() > 0)
  {
    // Yep. It's plugged in.
    if (TAIL_SERIAL.peek() == 't')
    {
      // Read out the 't' and sent it up.
      USB_COM_PORT << "Looks like the serial terminator is plugged into me. Sending up 't' aka ASCII 116.\n";
      TAIL_SERIAL.read();
      HEAD_SERIAL.write('t');
    }
  }
  else
  {
    USB_COM_PORT << "No serial terminator\n";
  }
  
  // Take 100ms to send up any other serial data.
  long startTime = millis();
  while ((millis() - startTime) < 100)
  {
    if (TAIL_SERIAL.available() > 0)
    {
      byte data = TAIL_SERIAL.read();
      USB_COM_PORT << "Recieved " << data << " on tail. Sending it up.\n";
      HEAD_SERIAL.write(data); 
    }
  }
  USB_COM_PORT << "\n";
}

/************************************************************************************
  processRunPIDCommand(): Runs the PID loop if the kill switch is still pressed.
 ***********************************************************************************/
void processRunPIDCommand()
{
  TAIL_SERIAL.write('p');   
  if (killSwitchPressed)
  {     
    move(); 
  }
}

/************************************************************************************
  processCountModulesCommand(): Tells the head this modules exists, records myModuleNumber.
 ***********************************************************************************/
void processCountModulesCommand()
{
  USB_COM_PORT.print("I am module number... ");
  while (HEAD_SERIAL.available() < 1);
  
  // Record my module number
  myModuleNumber = HEAD_SERIAL.read();
  EEPROM.write(0, myModuleNumber);
  
  // Tell the next module it's number
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write('n');
  TAIL_SERIAL.write(myModuleNumber + 1);
  
  // Tell the head I exist.
  HEAD_SERIAL.write(myModuleNumber);  
  USB_COM_PORT.println(myModuleNumber);  
}

/************************************************************************************
  processNewSettingsAndSetpoints(): Recieves new settings and setpoints from the head
 ***********************************************************************************/
void processNewSettingsAndSetpoints()
{
  char settings[125];
  
  // Get data array from upstream. Store in an array.
  HEAD_SERIAL.setTimeout(30);
  if (HEAD_SERIAL.readBytes(settings, 125) < 125)
  {
    USB_COM_PORT.println("ERROR: Missing settings array");
    return;
  }
  
  // Send data array downstream.
  TAIL_SERIAL.write('s');
  TAIL_SERIAL.write((byte*)settings, 125);

  // Copy new setpoints [Setting bytes 0 to 59]
  for (int i = 0; i < 5; ++i)
  {
    char newHorzAngle = (char)settings[(myModuleNumber-1) * 5 + i];
    char newVertAngle = (char)settings[(myModuleNumber-1) * 5 + i + 30];
    
    int newHorzSetpoint = map(newHorzAngle,0,255,horzLowCalibration[i],horzHighCalibration[i]);
    int newVertSetpoint = map(newVertAngle,0,255,vertLowCalibration[i],vertHighCalibration[i]);
    
    // If this is a new setpoint, reset the PID timout timer
    if (newHorzSetpoint != horzSetpoints[i])
    {
      horzTimerArray[i] = millis();
      horzSetpoints[i] = newHorzSetpoint;      
    }
    if (newVertSetpoint != vertSetpoints[i])
    {
      vertTimerArray[i] = millis();
      vertSetpoints[i] = newVertSetpoint;
    }
  }
  
  // Change the lights
  int lightByte = 60 + myModuleNumber - 1;
  digitalWrite(LED[0], (settings[lightByte] & B00000001) == B00000001);
  digitalWrite(LED[1], (settings[lightByte] & B00000010) == B00000010);
  digitalWrite(LED[2], (settings[lightByte] & B00000100) == B00000100);
  digitalWrite(LED[3], (settings[lightByte] & B00001000) == B00001000);
  digitalWrite(LED[4], (settings[lightByte] & B00010000) == B00010000);

  // Check the kill switch [Setting bit 70.0]
  killSwitchPressed = (settings[70] & B00000001) > 0;
  if (killSwitchPressed == false)
  {
    for(int i=0; i < 5; i++)
    {
      analogWrite(VERT_ACTUATOR[i], 0);
      analogWrite(HORZ_ACTUATOR[i], 0);
    }
    analogWrite(MOTOR_CONTROL, 0);
  }
  
  // Copy new motor speed [Setting byte 72]
  motorSpeed = settings[72];
  //USB_COM_PORT.print(motorSpeed);
  //USB_COM_PORT.print("\t");
  //USB_COM_PORT.println(horzAngleArray[4]);
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

 /************************************************************************************
  processDiagnosticsCommand(): Responds to the head's request to get diagnsotic info.
 ***********************************************************************************/

void processDiagnosticsCommand()
{
  char packetType[1];

  // Get data array from upstream. Store in an array.
  HEAD_SERIAL.setTimeout(30);
  if (HEAD_SERIAL.readBytes(packetType, 1) < 1)
  {
    USB_COM_PORT.println("ERROR: Missing diagnostics packet type");
    return;
  }

  switch (packetType[0])
  {
  case 1:
    sendHeadAndModuleDiagnostics();
    break;
  case 2:
    sendHorzAngleDiagnostics();
    break;
  case 3:
    sendVertAngleDiagnostics();
    break;
  case 4:
    sendHorzCalibrationDiagnostics();
    break;
  case 5:
    sendVertCalibrationDiagnostics();
    break;
  default:
    USB_COM_PORT.println("ERROR: Invalid diagnostics packet type");
    return;
  }

  // Tell next module to send diagnostics
  TAIL_SERIAL.write('d'); 
  TAIL_SERIAL.write(packetType[0]);
}

/************************************************************************************
 * sendHeadAndModuleDiagnostics(): Sends general information about the module. Not vertibrae.
 ***********************************************************************************/
void sendHeadAndModuleDiagnostics()
{
  int batteryVoltage = map(analogRead(BAT_LEVEL_24V),0,1023,0,25000);

  byte data[6];
  data[0] = highByte(batteryVoltage);
  data[1] = lowByte(batteryVoltage);
  data[2] = 0;
  data[3] = motorSpeed;
  data[4] = 0;
  data[5] = 0;

  HEAD_SERIAL.write(data, 6);
}

/************************************************************************************
 * sendHorzAngleDiagnostics(): 
 ***********************************************************************************/
void sendHorzAngleDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    int sensorValue = analogRead(HORZ_POS_SENSOR[i]);
    data[i * 4 + 0] = lowByte(horzSetpoints[i]);
    data[i * 4 + 1] = lowByte(horzSetpoints[i]);
    data[i * 4 + 2] = highByte(sensorValue);
    data[i * 4 + 3] = lowByte(sensorValue);
  }
  HEAD_SERIAL.write(data, 20);
}

/************************************************************************************
 * sendVertAngleDiagnostics(): 
 ***********************************************************************************/
void sendVertAngleDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    int sensorValue = analogRead(VERT_POS_SENSOR[i]);
    data[i * 4 + 0] = lowByte(vertSetpoints[i]);
    data[i * 4 + 1] = lowByte(vertSetpoints[i]);
    data[i * 4 + 2] = highByte(sensorValue);
    data[i * 4 + 3] = lowByte(sensorValue);
  }
  HEAD_SERIAL.write(data, 20);
}

/************************************************************************************
 * sendHorzCalibrationDiagnostics(): 
 ***********************************************************************************/
void sendHorzCalibrationDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    data[i * 4 + 0] = highByte(horzHighCalibration[i]);
    data[i * 4 + 1] = lowByte(horzHighCalibration[i]);
    data[i * 4 + 2] = highByte(horzLowCalibration[i]);
    data[i * 4 + 3] = lowByte(horzLowCalibration[i]);
  }
  HEAD_SERIAL.write(data, 20);
}

/************************************************************************************
 * sendVertCalibrationDiagnostics(): 
 ***********************************************************************************/
void sendVertCalibrationDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    // Note: we're sending the straight array for now
    // instead of the calibration. It's more useful.
    data[i * 4 + 0] = highByte(vertStraightArray[i]);
    data[i * 4 + 1] = lowByte(vertStraightArray[i]);
    data[i * 4 + 2] = highByte(0);
    data[i * 4 + 3] = lowByte(0);
  }
  HEAD_SERIAL.write(data, 20);
}


/************************************************************************************
  processLongMotorPulseCommand(): Pulses the first module motor on and off. Used for 
                                  jaw and head lift actuation.
 ***********************************************************************************/

void processLongMotorPulseCommand()
{ 
  // This command is only for the first module
  if (myModuleNumber != 1)
    return;
    
  USB_COM_PORT.print("Turning motor on for 2500ms\n");
  analogWrite(MOTOR_CONTROL, JAW_MOTOR_SPEED);
  delay(2500);
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT.print("Turning motor off\n");  
}

/************************************************************************************
  processShortMotorPulseCommand(): Turns the first module motor on and off. Used for 
                                   controlling the jaw and head lift in manual mode.
 ***********************************************************************************/
 
void processShortMotorPulseCommand()
{ 
  // This command is only for the first module
  if (myModuleNumber != 1)
    return;
    
  USB_COM_PORT.print("Turning motor on for 200ms\n");
  analogWrite(MOTOR_CONTROL, motorSpeed);
  delay(200);
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT.print("Turning motor off\n");  
}

/************************************************************************************
  calculateSensorDeadBands(): Based on the calibration, map the deadbands from angle space (0-255) to
                              sensor space (low to high calibration)
 ***********************************************************************************/
void calculateSensorDeadBands()
{
  for (int i = 0; i < 5; ++i)
  {
    horzSensorDeadbands[i] = map(horzAngleDeadband, 0, 255, horzLowCalibration[i], horzHighCalibration[i]);
    vertSensorDeadbands[i] = map(vertAngleDeadband, 0, 255, vertLowCalibration[i], vertHighCalibration[i]);
  }
}

/************************************************************************************
  move(): Modulates the values to achieve position set points (Runs the PID)
 ***********************************************************************************/

void move()
{
  //set if any segment moved, so we can turn off motor when setpoint reached
  boolean moved = false;
  int goal;
  int currentAngle;

  for (int i=0; i < 5; i++)
  {
    //////////////////////////
    ////// HORIZONTAL ////////
    if (horzSetpoints[i] >= 0)
    {
      PIDcontrollerHorizontal[i].setSetPoint(horzSetpoints[i]);
      currentAngle = analogRead(HORZ_POS_SENSOR[i]);
      
      // If we are have not reached the deadband and haven't timed out
      if ((abs(currentAngle - horzSetpoints[i]) > horzSensorDeadbands[i]) &&
          (millis() - horzTimerArray[i]) < horzActuatorTimeout)
      {
        // Update the acutator PID output
        analogWrite(MOTOR_CONTROL, motorSpeed);
        PIDcontrollerHorizontal[i].updateOutput();
        moved = true;
      }
      // We are in the deadband or we took too long to get there    
      else
      {
        analogWrite(HORZ_ACTUATOR[i],0);
      }
    }
    // Setpoint is uninitialized (-1) turn actuator off
    else
    {
      analogWrite(VERT_ACTUATOR[i], 0);
    }

    ///////////////////////    
    ////// VERTICAL ///////
    if (vertSetpoints[i] >= 0)
    {      
      PIDcontrollerVertical[i].setSetPoint(vertSetpoints[i]);
      currentAngle = analogRead(VERT_POS_SENSOR[i]);
      
      // If we are have not reached the deadzone and haven't timed out
      if ((abs(currentAngle - vertSetpoints[i]) > vertSensorDeadbands[i]) &&
          (millis() - vertTimerArray[i]) < vertActuatorTimeout)
      {
        // Update the acutator PID output        
        analogWrite(MOTOR_CONTROL, motorSpeed);
        PIDcontrollerVertical[i].updateOutput();
        moved = true;
      }
      // We are in the deadband or we took too long to get there
      else
      {
        analogWrite(VERT_ACTUATOR[i], 0);
      }
    }
    // Setpoint is uninitialized (-1) turn actuator off
    else
    {
      analogWrite(VERT_ACTUATOR[i], 0);
    }
  }
  
  if(moved == false)
  {
    analogWrite(MOTOR_CONTROL, 0);
  }
}// end move()


/**************************************************************************************
  processHorzCalibrateCommand(): Tells the next module to calibrate then calibrates this one.
 *************************************************************************************/
void processHorzCalibrateCommand()
{
  TAIL_SERIAL.write('c');

  // Stop any current motion
  for(int i=0; i < 5; i++)
  {
    analogWrite(VERT_ACTUATOR[i], 0);
    analogWrite(HORZ_ACTUATOR[i], 0);
  }
  analogWrite(MOTOR_CONTROL, 0);
  
  calibrateHorizontal();
} // end processHorzCalibrateCommand

/**************************************************************************************
  calibrateHorizontal(): Records horizontal sensor values at the hard limits of each vertebrae. Saves to EEPROM.
 *************************************************************************************/
void calibrateHorizontal()
{
  // HIGH: Move to side when the solenoid selection signal is HIGH
  analogWrite(MOTOR_CONTROL, CALIBRATE_MOTOR_SPEED);
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(4000);
  // HIGH: Stop
  analogWrite(MOTOR_CONTROL,0);
  for (int i=0;i<5;i++)
  {
    analogWrite(HORZ_ACTUATOR[i],0);
  }
  delay(1000);
  // HIGH: Record Values
  for(int i=0;i<5;i++)
  {
    horzHighCalibration[i] = analogRead(HORZ_POS_SENSOR[i]);
  }

  // LOW: Move to side when the selection signal is LOW
  analogWrite(MOTOR_CONTROL, CALIBRATE_MOTOR_SPEED);
  for(int i=0;i<5;i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], LOW);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(4000);
  // LOW: Stop
  analogWrite(MOTOR_CONTROL,0);
  for(int i=0;i<5;i++)
  {
    analogWrite(HORZ_ACTUATOR[i],0);
  }
  delay(1000);
  // LOW: Record Values
  for(int i=0;i<5;i++)
  {
    horzLowCalibration[i] = analogRead(HORZ_POS_SENSOR[i]);
  }

  // Adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++)
  {
    if(horzHighCalibration[i]>horzLowCalibration[i])
    {
      PIDcontrollerHorizontal[i].setEven(true);
    }
    else
    {
      int temp = horzHighCalibration[i];
      horzHighCalibration[i] = horzLowCalibration[i];
      horzLowCalibration[i] = temp;
      PIDcontrollerHorizontal[i].setEven(false);
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i=0;i<5;i++){ 
    EEPROM.write(i*5+2,lowByte(horzHighCalibration[i]));
    EEPROM.write(i*5+3,highByte(horzHighCalibration[i]));
    EEPROM.write(i*5+4,lowByte(horzLowCalibration[i]));
    EEPROM.write(i*5+5,highByte(horzLowCalibration[i]));
    EEPROM.write(i*5+6,PIDcontrollerHorizontal[i].getEven());
  }
   
  // Print out calibration
  USB_COM_PORT.println("");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(horzHighCalibration[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("HIGH ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(horzLowCalibration[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("RANGE ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(horzHighCalibration[i] - horzLowCalibration[i]);
    USB_COM_PORT.print(" ");
  }
  USB_COM_PORT.println("LOW ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(PIDcontrollerHorizontal[i].getEven(),BIN);
    USB_COM_PORT.print("    ");
  }
  USB_COM_PORT.println("EVEN");
  
}//end calibrateHorizontal()


/**************************************************************************************
  processVertCalibrateCommand(): Tells the next module to calibrate vertical then calibrates this one.
 *************************************************************************************/
void processVertCalibrateCommand()
{
  TAIL_SERIAL.write('v');

  // Stop any current motion
  for(int i=0; i < 5; i++)
  {
    analogWrite(VERT_ACTUATOR[i], 0);
    analogWrite(HORZ_ACTUATOR[i], 0);
  }
  analogWrite(MOTOR_CONTROL, 0);
  
  calibrateVertical();
} // end processVertCalibrateCommand


/**************************************************************************************
  calibrateVertical(): Records vertical sensor values at the hard limits of each 
                       vertebrae. 
                       Saves to EEPROM.
 *************************************************************************************/
void calibrateVertical()
{
  analogWrite(MOTOR_CONTROL, CALIBRATE_MOTOR_SPEED);

  // for vertical calibration don't run actuators to their extremes all in the same 
  // direction otherwise will end up with a giant U that will tip over
  for (int i = 0; i < 5; i++)
  {
    if (myModuleNumber%2 == 0)
    {
      //even numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
    else
    {
      //odd numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
  }

  //ensure there's enough time for actuators to reach their maximum
  //before turning off the motor and killing signal to actuators
  delay(4000);
  analogWrite(MOTOR_CONTROL, 0);
  for (int i=0;i<5;i++)
  {
    analogWrite(VERT_ACTUATOR[i], 0);
  }

  // Record 1st set of values
  for(int i=0; i<5; i++)
  {
    vertHighCalibration[i] = analogRead(VERT_POS_SENSOR[i]);
  }

  //Now go to the other extremes
  analogWrite(MOTOR_CONTROL, CALIBRATE_MOTOR_SPEED);
  for (int i=0; i<5; i++)
  {
    if (myModuleNumber%2 == 0)
    {
      //even numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
    else
    {
      //odd numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
  }

  // ensure there's enough time for actuators to reach their maximum
  // before turning off the motor and killing signal to actuators
  delay(4000);
  analogWrite(MOTOR_CONTROL, 0);
  for (int i=0;i<5;i++)
  {
    analogWrite(VERT_ACTUATOR[i],0);
  }

  // Record 2nd set of values
  for (int i=0; i<5; i++)
  {
    vertLowCalibration[i] = analogRead(VERT_POS_SENSOR[i]);
  }
  
  // Adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++)
  {
    if(vertHighCalibration[i]>vertLowCalibration[i])
    {
      PIDcontrollerVertical[i].setEven(false);
    }
    else
    {
      int temp = vertHighCalibration[i];
      vertHighCalibration[i] = vertLowCalibration[i];
      vertLowCalibration[i] = temp;
      PIDcontrollerVertical[i].setEven(true);
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i=0;i<5;i++){ 
    EEPROM.write(i*5+2, lowByte(vertHighCalibration[i]));
    EEPROM.write(i*5+3, highByte(vertHighCalibration[i]));
    EEPROM.write(i*5+4, lowByte(vertLowCalibration[i]));
    EEPROM.write(i*5+5, highByte(vertLowCalibration[i]));
    EEPROM.write(i*5+6, PIDcontrollerVertical[i].getEven());
  }
   
  // Print out calibration
  USB_COM_PORT.println("");
  for (int i=0; i<5; i++)
  {
    USB_COM_PORT.print(vertHighCalibration[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("HIGH ");
  for (int i=0; i<5; i++)
  {
    USB_COM_PORT.print(vertLowCalibration[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("LOW ");
  for(int i=0; i<5; i++)
  {
    USB_COM_PORT.print(vertHighCalibration[i] - vertLowCalibration[i]);
    USB_COM_PORT.print(" ");
  }
  USB_COM_PORT.println("RANGE ");
  for (int i=0; i<5; i++)
  {
    USB_COM_PORT.print(PIDcontrollerVertical[i].getEven(), BIN);
    USB_COM_PORT.print("    ");
  }
  USB_COM_PORT.println("EVEN");

}//end calibrateVertical()

/**************************************************************************************
  readSensors():
 *************************************************************************************/
void readSensors()
{
  int sensorVal = 0;

  for(int i=0;i<5;i++)
  {
    // Get the current raw value
    sensorVal = analogRead(HORZ_POS_SENSOR[i]);

    USB_COM_PORT.print("Horizontal sensor ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(": current position ");
    USB_COM_PORT.println(sensorVal);
  }

  for(int i=0;i<5;i++)
  {
    // Get the current raw value
    sensorVal = analogRead(VERT_POS_SENSOR[i]);

    USB_COM_PORT.print("Vertical sensor ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(": current position ");
    USB_COM_PORT.print(sensorVal);
    USB_COM_PORT.print(", straight is ");
    USB_COM_PORT.println(vertStraightArray[i]);
  }
}


/**************************************************************************************
  saveVerticalPosition():
 *************************************************************************************/
void saveVerticalPosition()
{
  for(int i=0;i<5;i++)
  {
    // Get the current raw value
    vertStraightArray[i] = analogRead(VERT_POS_SENSOR[i]);
    // Set low and high ranges as +/- 100 of current position
    vertLowCalibration[i] = vertStraightArray[i] - 100;
    vertHighCalibration[i] = vertStraightArray[i] + 100;

    USB_COM_PORT.print("Vertical sensor ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(": current position ");
    USB_COM_PORT.print(vertStraightArray[i]);
    USB_COM_PORT.print(", low limit ");
    USB_COM_PORT.print(vertLowCalibration[i]);
    USB_COM_PORT.print(", high limit ");
    USB_COM_PORT.println(vertHighCalibration[i]);
  }

  // Save current vertical position to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i=0;i<5;i++)
  {
    // save current position as straight
    EEPROM.write(i*5+VERTICAL_STRAIGHT_ADDRESS, lowByte(vertStraightArray[i]));
    EEPROM.write(i*5+VERTICAL_STRAIGHT_ADDRESS+1, highByte(vertStraightArray[i]));
  }
}


/**************************************************************************************
  manualControl(): Allows for manual actuator control over the USB_SERIAL_PORT
 *************************************************************************************/
void manualControl()
{
  boolean manual = true;
  char byteIn = 'z';
  int segSelect = 0;
  boolean motor = false;
  int actuationDelay = 200;
  char delaySetting = 'm';
  byte newNum;

    displayMenu();

  while(manual == true)
  {
    if(Serial.available() > 0){
      byteIn = USB_COM_PORT.read();
      
      switch(byteIn)
      {
        case 'c':
          USB_COM_PORT.print("\nRunning horz calibration...\n");
          calibrateHorizontal();
          break;
        case 'v':
          USB_COM_PORT.print("\nRunning vert calibration...\n");
          calibrateVertical();
          break;
        case 'p':
          readSensors();
          break;
        case 'r':
          USB_COM_PORT.print("\nSaving current vertical position\n");
          saveVerticalPosition();
          break;
        case '1':
          segSelect = 0;
          StopMov();
          USB_COM_PORT.print("Seg1\n");
          motor = false;
          break;
        case '2':
          segSelect = 1;
          StopMov();
          USB_COM_PORT.print("Seg2\n");
          motor = false;
          break;
        case '3':
          segSelect = 2;
          StopMov();
          USB_COM_PORT.print("Seg3\n");
          motor = false;
          break;
        case '4':
          segSelect = 3;
          StopMov();
          USB_COM_PORT.print("Seg4\n");
          motor = false;
          break;
        case '5':
          segSelect = 4;
          StopMov();
          USB_COM_PORT.print("Seg5\n");
          motor = false;
          break;

        case 'd':
          StopMov();
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
          break;

        case 'l':
          if(motor == false)
          {
            StopMov();
            analogWrite(5, motorSpeed);
            analogWrite(HORZ_ACTUATOR[segSelect], 255);
            digitalWrite(31-segSelect, HIGH);
            USB_COM_PORT.print("l dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break;

        case 'k':
          if(motor == false)
          {
            StopMov();
            analogWrite(MOTOR_CONTROL, motorSpeed);
            analogWrite(HORZ_ACTUATOR[segSelect], 255);
            digitalWrite(31-segSelect,LOW);
            USB_COM_PORT.print("k dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break;

        case 'i':
          if(motor == false)
          {
            StopMov();
            analogWrite(MOTOR_CONTROL, motorSpeed);
            analogWrite(VERT_ACTUATOR[segSelect], 255);
            digitalWrite(26-segSelect, LOW);
            USB_COM_PORT.print("i dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break; 

        case 'o':
          if(motor == false)
          {
            analogWrite(MOTOR_CONTROL, motorSpeed);
            analogWrite(VERT_ACTUATOR[segSelect], 255);
            digitalWrite(26-segSelect, HIGH);
            USB_COM_PORT.print("o dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break;

        case 'n':
          for (int i=0; i<5; i++)
          {
              digitalWrite(LED[i], HIGH);
          }
          break;

        case 'm':
          for (int i=0; i<5; i++)
          {
              digitalWrite(LED[i], LOW);
          }
          break;

        case 's':
          StopMov();
          USB_COM_PORT.print("STOPPED\n");
          break;
          
        
        case 'u':
          USB_COM_PORT << "Manually program myModuleNumber (0-9)...\n";
          while(USB_COM_PORT.available() < 1);
          newNum = USB_COM_PORT.read() - 48;
          if (newNum < 0 || newNum > 9)
          {
            USB_COM_PORT << "Invalid\n";
            break;
          }
          USB_COM_PORT << "Now set to " << newNum << "\n";
          EEPROM.write(MY_MODULE_NUMBER_ADDRESS, newNum);
          myModuleNumber = newNum;
          break;        

        case 'e':
            displayMenu();
            break;

        case 'q':
          manual = false;
          HEAD_SERIAL.flush();
          TAIL_SERIAL.flush();
          break;
      }//end switch
    }//end if serial

    byteIn = 'z';
  }
  USB_COM_PORT.println("\nManual Control mode exited");
}


/**************************************************************************************
  displayMenu():
 *************************************************************************************/
void displayMenu()
{
    USB_COM_PORT.print("\nManual Control mode entered\n");
    USB_COM_PORT.print("commands: 1-5 to select vertebrae\n");
    USB_COM_PORT.print("          k/l - horizontal actuation\n");
    USB_COM_PORT.print("          i/o - vertical actuation\n");
    USB_COM_PORT.print("          d* - adjust actuation delay, where *=s(small),m(medium),l(large)\n");
    USB_COM_PORT.print("          c - calibrate horizontal\n");
    USB_COM_PORT.print("          p - print raw values of position sensors\n");
    USB_COM_PORT.print("          r - save current vertical position as straight\n");
    USB_COM_PORT.print("          v - calibrate verticals\n");
    USB_COM_PORT.print("          n/m - all leds on/off\n");
    USB_COM_PORT.print("          s - stop motor\n");
    USB_COM_PORT.print("          u - manually set myModuleNumber\n");    
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n\n");
}


/**************************************************************************************
  StopMov(): In manual mode, stops movement of all actuators.
 *************************************************************************************/
void StopMov()
{
  
  /*for (int i=0; i<5; i++)
  {
    analogWrite(HORZ_ACTUATOR[i], 0);
    analogWrite(VERT_ACTUATOR[i], 0);
  }*/
  analogWrite(2,0);
  analogWrite(3,0);
  analogWrite(4,0);
  analogWrite(5,0);
  analogWrite(6,0);
  analogWrite(7,0);
  analogWrite(8,0);
  analogWrite(9,0);
  analogWrite(10,0);
  analogWrite(11,0);
  analogWrite(12,0);
  analogWrite(13,0);

  digitalWrite(31,LOW);
  digitalWrite(30,LOW);
  digitalWrite(29,LOW);
  digitalWrite(28,LOW);
  digitalWrite(27,LOW);

  digitalWrite(26,LOW);
  digitalWrite(25,LOW);
  digitalWrite(24,LOW);
  digitalWrite(23,LOW);
  digitalWrite(22,LOW);

  return;
}








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

// We are using an interrupt for the PID. this interrupt could come in any time, possibly in the middle
// of communication. To ensure that we don't lose data, make the serial buffer size larger than the packets.
#include "HardwareSerial.cpp"
#if SERIAL_BUFFER_SIZE != 128
  #error "ERROR: Serial buffer must be 128 bytes. Please modify your arduino program files."
  // Arduino Folder\hardware\arduino\cores\arduino
#endif 

#include  <EEPROM.h>
#include "modulePins.h"
#include "PIDcontrol.h"
#include "OneWireNXP.h"

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

// Target sensor values for the actuators
// -1 means un initialized
int horzSensorSetpoints[] = {-1,-1,-1,-1,-1};
int vertSensorSetpoints[] = {-1,-1,-1,-1,-1};
int horzAngleSetpoints[] = {255,255,255,255,255};
int vertAngleSetpoints[] = {255,255,255,255,255};

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

// We configure the deadbands in angle space (0-254). These arrays are a mapping of these
// values to each sensor space. They are calculated by calling calculateDeadbands()
int horzSensorDeadbands[] = {-1,-1,-1,-1,-1};
int vertSensorDeadbands[] = {-1,-1,-1,-1,-1};

byte myModuleNumber = 0;                // My position in the module chain (1,2,3...)
boolean iAmLastModule = false;          // If we are the last module don't read tail serial
byte killSwitchPressed = false;         // If the kill switch isn't pressed. Don't move.
boolean runPidLoop = false;             // Set to true if you want to run pid on interrupt interval.
byte runtimeMotorSpeed = 0;             // Runtime analog output motor speed. Set by joystick.
const byte horzAngleDeadband = 10;      // The horizontal deadband, 0 to 254 range
const byte vertAngleDeadband = 10;      // The vertical deadband, 0 to 254 range
const int horzActuatorTimeout = 3000;   // Horizontal actuators have this many ms to get to setpoint
const int vertActuatorTimeout = 3000;   // Vertical actuators have this many ms to get to setpoint
const byte calibrateMotorSpeed = 150;   // Motor speed for vertical and horz calibration.
const byte jawMotorSpeed = 90;          // Motor speed for jaw open/close. (the jaw uses the module 1 motor)
const byte manualMotorSpeed = 200;      // Motor speed for manual mode operations.
const int pidLoopTime = 30;             // We execute the pid loop on this interval (ms)

// Variables for manual mode
int manualActuationDelay = 200;
int manualVertibraeSelect = 0;


// PID Controllers for the horizontal actuators
// Declaration of horizontal PID controllers, default even and odd values applied here
// but get updated during calibration
PIDcontrol PIDcontrollerHorizontal[] = {
  PIDcontrol(HORZ_POS_SENSOR[0], HORZ_ACTUATOR_CTRL[0], HORZ_ACTUATOR[0]),
  PIDcontrol(HORZ_POS_SENSOR[1], HORZ_ACTUATOR_CTRL[1], HORZ_ACTUATOR[1]),
  PIDcontrol(HORZ_POS_SENSOR[2], HORZ_ACTUATOR_CTRL[2], HORZ_ACTUATOR[2]),
  PIDcontrol(HORZ_POS_SENSOR[3], HORZ_ACTUATOR_CTRL[3], HORZ_ACTUATOR[3]),
  PIDcontrol(HORZ_POS_SENSOR[4], HORZ_ACTUATOR_CTRL[4], HORZ_ACTUATOR[4]),
};

// PID Controllers for the vertical actuators
// Declaration of vertical PID controllers, default even and odd values only apply to
// odd numbered modules
PIDcontrol PIDcontrollerVertical[] = {
  PIDcontrol(VERT_POS_SENSOR[0], VERT_ACTUATOR_CTRL[0], VERT_ACTUATOR[0]),
  PIDcontrol(VERT_POS_SENSOR[1], VERT_ACTUATOR_CTRL[1], VERT_ACTUATOR[1]),
  PIDcontrol(VERT_POS_SENSOR[2], VERT_ACTUATOR_CTRL[2], VERT_ACTUATOR[2]),
  PIDcontrol(VERT_POS_SENSOR[3], VERT_ACTUATOR_CTRL[3], VERT_ACTUATOR[3]),
  PIDcontrol(VERT_POS_SENSOR[4], VERT_ACTUATOR_CTRL[4], VERT_ACTUATOR[4]),
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
  for(int i=0;i<8;i++)
  {
    pinMode(LED[i],OUTPUT);
  }
  
  // Set the constants of the PID controllers
  for(int i=0;i<5;i++)
  {
    PIDcontrollerHorizontal[i].setConstants(50,0,0);
    PIDcontrollerVertical[i].setConstants(50,0,0);
  }

  // Load previous calibration from EEPROM
  setModuleNumber(EEPROM.read(MY_MODULE_NUMBER_ADDRESS));
  
  for(int i=0;i<5;i++)
  { 
    horzHighCalibration[i] = EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS) + 
                            (EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS + 1) << 8);
    horzLowCalibration[i] =  EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS + 2) +
                            (EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS + 3) << 8);

    vertStraightArray[i] = EEPROM.read(i*5 + VERTICAL_STRAIGHT_ADDRESS)
                           + (EEPROM.read(i*5 + VERTICAL_STRAIGHT_ADDRESS+1) << 8);
                           
    vertHighCalibration[i] = EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS) + 
                            (EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS + 1) << 8);
    vertLowCalibration[i] =  EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS + 2) +
                            (EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS + 3) << 8);   
  }
  
  calculateSensorDeadbands();

  // Print out the initialization information
  USB_COM_PORT.print("\nHi I'm Titanoboa, MODULE #: ");
  USB_COM_PORT.println(myModuleNumber, DEC);

  // Check if I'm the last module
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write(200);
  delay(10);
  boolean avail = (TAIL_SERIAL.available() == 1);
  int data = TAIL_SERIAL.read();
  if (avail && (data == 200))
  {
    iAmLastModule = true;
    USB_COM_PORT << "... and I'm the last module! (" << data << ")\n\n";
  }
  else
  {
    USB_COM_PORT << "Not last. (" << data << ")\n\n";    
  }
 
  printBatteryVoltage();
  printCalibrationValues();
  
  clearSerialBuffer(HEAD_SERIAL);
  clearSerialBuffer(TAIL_SERIAL);
  
  // AVR code to enable overflow interrupt on timer 1
  // Used for the pid interrupt
  TIMSK1 |= _BV(TOIE1);
  runPidLoop = true;
}

/***********************************************************************************
  loop(): Checks for Upstream Serial or USB Serial commands. Executes them.
 ***********************************************************************************/
 
void loop()
{ 
  // Check the USB port for command to enter manual mode
  if (USB_COM_PORT.available() > 0)
  {
    if (USB_COM_PORT.read() == 'M')
    {
      manualControl();
    }
  }

  // Check upstream Serial for commands from the head
  if (HEAD_SERIAL.available() > 0)
  {
    //letters in use: p, s, c, h, g, v, t
    char command = HEAD_SERIAL.read();
    switch (command)
    {
      case 's':
        processNewSettingsAndSetpoints();
        acknowledgeCommand();
        break;
      case 'd':
        processDiagnosticsCommand();
        break;
      case 'c':
        processCalibrateCommand();
        acknowledgeCommand();
        break;
      case 'h':
        processJawMovingMotorPulseCommand();
        break;
      case 'g':
        processHeadManualModeMotorPulseCommand();
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
    byte newHorzAngle = (char)settings[(myModuleNumber-1) * 5 + i];
    byte newVertAngle = (char)settings[(myModuleNumber-1) * 5 + i + 30];
      
    setHorzAngleSetpoint(i, newHorzAngle);
    setVertAngleSetpoint(i, newVertAngle);
  }
  
  // Change the lights
  int lightByte = 60 + myModuleNumber - 1;
  digitalWrite(LED[0], (settings[lightByte] & B00000001) == B00000001);
  digitalWrite(LED[1], (settings[lightByte] & B00000010) == B00000010);
  digitalWrite(LED[2], (settings[lightByte] & B00000100) == B00000100);
  digitalWrite(LED[3], (settings[lightByte] & B00001000) == B00001000);
  digitalWrite(LED[4], (settings[lightByte] & B00010000) == B00010000);

  // Check the kill switch [Setting bit 70.0]
  boolean newKillSwitchPressed = (settings[70] & B00000001) > 0;
  if (newKillSwitchPressed == false)
  {
    // Stop all movement when the kill switch is released
    StopMovement();
  }
  if (newKillSwitchPressed == true && killSwitchPressed == false)
  {
    // Reset timeout timers when kill switch is pressed
    for (int i = 0; i < 5; ++i)
    {
      horzTimerArray[i] = millis();
      vertTimerArray[i] = millis();
    }
  }
  killSwitchPressed = newKillSwitchPressed;
  
  // Copy new motor speed [Setting byte 72]
  runtimeMotorSpeed = settings[72];
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
    sendAngleAndSetpointDiagnostics();
    break;
  case 3:
    sendHorzCalibrationDiagnostics();
    break;
  case 4:
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
  int hydraulicPressure = map(analogRead(PRESSURE_SENSOR),102,921,0,2000);
  
  byte data[6];
  data[0] = highByte(batteryVoltage);
  data[1] = lowByte(batteryVoltage);
  data[2] = 0;
  data[3] = runtimeMotorSpeed;
  data[4] = highByte(hydraulicPressure);
  data[5] = lowByte(hydraulicPressure);

  HEAD_SERIAL.write(data, 6);
}

/************************************************************************************
 * sendHorzAngleDiagnostics(): 
 ***********************************************************************************/
void sendAngleAndSetpointDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    byte horzAngle = getCurrentHorizontalAngle(i);
    byte vertAngle = getCurrentVerticalAngle(i);
       
    data[i * 4 + 0] = horzAngleSetpoints[i];
    data[i * 4 + 1] = horzAngle;
    data[i * 4 + 2] = vertAngleSetpoints[i];
    data[i * 4 + 3] = vertAngle;
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
    data[i * 4 + 0] = highByte(vertHighCalibration[i]);
    data[i * 4 + 1] = lowByte(vertHighCalibration[i]);
    data[i * 4 + 2] = highByte(vertLowCalibration[i]);
    data[i * 4 + 3] = lowByte(vertLowCalibration[i]);
  }
  HEAD_SERIAL.write(data, 20);
}


/************************************************************************************
  processJawMovingMotorCommand(): Turns the first motor on the off so the jaw can 
                                  open or close.
 ***********************************************************************************/

void processJawMovingMotorPulseCommand()
{
  runPidLoop = false;
  
  // This command is only for the first module
  if (myModuleNumber != 1)
    return;
    
  USB_COM_PORT << "Turning motor on for 2500ms (speed = " << jawMotorSpeed << ")\n";
  analogWrite(MOTOR_CONTROL, jawMotorSpeed);
  delay(2500);
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT.print("Turning motor off\n");
  
  runPidLoop = true;
}

/************************************************************************************
  processShortMotorPulseCommand(): Turns the first module motor on and off so we can
                                   do manual mode on the head and jaw.
 ***********************************************************************************/
 
void processHeadManualModeMotorPulseCommand()
{ 
  runPidLoop = false;
  
  // This command is only for the first module
  if (myModuleNumber != 1)
    return;
    
  USB_COM_PORT.print("Turning motor on for 200ms\n");
  analogWrite(MOTOR_CONTROL, manualMotorSpeed);
  delay(400);
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT.print("Turning motor off\n");
  
  runPidLoop = true;
}

/************************************************************************************
  calculateSensorDeadBands(): Based on the calibration, map the deadbands from angle space (0-254) to
                              sensor space (low to high calibration)
 ***********************************************************************************/
void calculateSensorDeadbands()
{
  for (int i = 0; i < 5; ++i)
  {
    horzSensorDeadbands[i] = map(horzAngleDeadband, 0, 254, 0, horzHighCalibration[i] - horzLowCalibration[i]);
    vertSensorDeadbands[i] = map(vertAngleDeadband, 0, 254, 0, vertHighCalibration[i] - vertLowCalibration[i]);
  }
}


/************************************************************************************
  This is an AVR interrupt. On the Arduino MEGA 2560 it triggers every 2ms (different on other Arduinos)
  We are using this interrupt to the run pid loop on a fixed, consistant interval. 
 ***********************************************************************************/
ISR(TIMER1_OVF_vect, ISR_NOBLOCK)
{
  if (runPidLoop == false)
    return;
    
  static int count = 0;
  ++count;
  
  // Wait for "pidLoopTime" milliseconds to occur
  if (count > pidLoopTime / 2) // 2 means 2ms
  {    
    if (killSwitchPressed)
    {
      executePID(runtimeMotorSpeed, true, true); 
    }
    else
    {
      StopMovement();
    }      
    count = 0;
  }
}

/************************************************************************************
  executePID(): Modulates the values to achieve position set points (Runs the PID)
 ***********************************************************************************/
void executePID(byte motorSpeed, boolean doHorizontal, boolean doVertical)
{
  //set if any segment moved, so we can turn off motor when setpoint reached
  boolean moved = false;
  int goal;
  int currentAngle;

  for (int i=0; i < 5; i++)
  {
    //////////////////////////
    ////// HORIZONTAL ////////
    if (horzSensorSetpoints[i] >= 0 && doHorizontal)
    {
      PIDcontrollerHorizontal[i].setSetPoint(horzSensorSetpoints[i]);
      currentAngle = analogRead(HORZ_POS_SENSOR[i]);
      
      // If we are have not reached the deadband and haven't timed out
      if ((abs(currentAngle - horzSensorSetpoints[i]) > horzSensorDeadbands[i]) &&
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
    if (vertSensorSetpoints[i] >= 0 && doVertical)
    {      
      PIDcontrollerVertical[i].setSetPoint(vertSensorSetpoints[i]);
      currentAngle = analogRead(VERT_POS_SENSOR[i]);
      
      // If we are have not reached the deadzone and haven't timed out
      if ((abs(currentAngle - vertSensorSetpoints[i]) > vertSensorDeadbands[i]) &&
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
  manualStraightenHorizontal(): Makes titanoboa vertically straight as an arrow horizontally.
                                Important! Make sure pid is disabled when running this function.
 *************************************************************************************/
void manualStraightenHorizontal()
{
  USB_COM_PORT << "Straightening horizontal. ";
  
  // Set setpoints to halfway between low and high calibration values
  setHorzAngleSetpoint(0, 127);
  setHorzAngleSetpoint(1, 127);
  setHorzAngleSetpoint(2, 127);
  setHorzAngleSetpoint(3, 127);
  setHorzAngleSetpoint(4, 127);  
  manualGoToSetpoints();
}

/**************************************************************************************
  manualStraightenVertical(): For manual mode only. Makes titanoboa vertically straight.
                              Important! Make sure pid is disabled when running this function.
 *************************************************************************************/
void manualStraightenVertical()
{
  USB_COM_PORT << "Straightening vertical. ";
  
  setVertAngleSetpoint(0, 127);
  setVertAngleSetpoint(1, 127);
  setVertAngleSetpoint(2, 127);
  setVertAngleSetpoint(3, 127);
  setVertAngleSetpoint(4, 127);  
  manualGoToSetpoints();
}

/**************************************************************************************
  processCalibrateCommand(): Straightens. Tells the next module to calibrate then calibrates this one.
 *************************************************************************************/
void processCalibrateCommand()
{
  runPidLoop = false;
  char vertOrHorz[1];

  // Get data array from upstream. This will tell us if we're doing vertical or horizontal
  HEAD_SERIAL.setTimeout(30);
  if (HEAD_SERIAL.readBytes(vertOrHorz, 1) < 1)
  {
    USB_COM_PORT.println("ERROR: Missing vertical or horizontal byte");
    return;
  }
  if (vertOrHorz[0] != 'h' && vertOrHorz[0] != 'v')
  {
    USB_COM_PORT << "ERROR: Expected 'h' or 'v'. Got '" << vertOrHorz[0] << "'";
    return;
  }
  
  TAIL_SERIAL.write('c');
  TAIL_SERIAL.write(vertOrHorz[0]);
  
  // Stop any current motion
  StopMovement();
  
  // Run calibration
  if (vertOrHorz[0] == 'h')
  {
    USB_COM_PORT << "h";
    calibrateHorizontal();
  }
  if (vertOrHorz[0] == 'v')
  {
    calibrateVertical();
  }
  runPidLoop = true;
} // end processHorzCalibrateCommand



/**************************************************************************************
  calibrateHorizontal(): Records horizontal sensor values at the hard limits of each vertebrae. Saves to EEPROM.
 *************************************************************************************/
void calibrateHorizontal()
{
  USB_COM_PORT << "\nCalibrating Horizontal\n"; 
  
  USB_COM_PORT << "Contracting All... ";
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);  
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(4000);
  StopMovement(); 
  delay(1000);
  for(int i = 0; i < 5; i++)
  {
    horzHighCalibration[i] = analogRead(HORZ_POS_SENSOR[i]);
  }
  USB_COM_PORT << "Recorded\n";   

  USB_COM_PORT << "Extending All... ";
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], LOW);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(4000);
  StopMovement();
  delay(1000);
  for(int i = 0; i < 5; i++)
  {
    horzLowCalibration[i] = analogRead(HORZ_POS_SENSOR[i]);
  }
  USB_COM_PORT << "Recorded\n";    

  // Adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++)
  {
    if(horzHighCalibration[i]>horzLowCalibration[i])
    {
    }
    else
    {
      int temp = horzHighCalibration[i];
      horzHighCalibration[i] = horzLowCalibration[i];
      horzLowCalibration[i] = temp;
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i = 0; i < 5; i++)
  { 
    EEPROM.write(i*5+2,lowByte(horzHighCalibration[i]));
    EEPROM.write(i*5+3,highByte(horzHighCalibration[i]));
    EEPROM.write(i*5+4,lowByte(horzLowCalibration[i]));
    EEPROM.write(i*5+5,highByte(horzLowCalibration[i]));
  }

  manualStraightenHorizontal();
  USB_COM_PORT << "Calibration Done\n";
  
  calculateSensorDeadbands();
  printCalibrationValues();
  
}//end calibrateHorizontal()


/**************************************************************************************
  calibrateVertical(): Records vertical sensor values at the hard limits of each 
                       vertebrae. 
                       Saves to EEPROM.
 *************************************************************************************/
void calibrateVertical()
{
  USB_COM_PORT << "Calibrating Vertical\n";
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);

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
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);
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
    if(vertHighCalibration[i] < vertLowCalibration[i])
    {
      int temp = vertHighCalibration[i];
      vertHighCalibration[i] = vertLowCalibration[i];
      vertLowCalibration[i] = temp;
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i=0;i<5;i++)
  { 
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 0, lowByte(vertHighCalibration[i]));
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 1, highByte(vertHighCalibration[i]));
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 2, lowByte(vertLowCalibration[i]));
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 3, highByte(vertLowCalibration[i]));
  }
  manualStraightenVertical();
  USB_COM_PORT << "Calibration Done\n";
  
  calculateSensorDeadbands();
  printCalibrationValues();
  
}//end calibrateVertical()


/**************************************************************************************
  getCurrentHorizontalAngle(): Get horizontal vertibrae value in 0-254 angle space
 *************************************************************************************/
byte getCurrentHorizontalAngle(int i)
{
  return map(analogRead(HORZ_POS_SENSOR[i]), horzLowCalibration[i], horzHighCalibration[i], 0, 254);
}


/**************************************************************************************
  getCurrentVerticalAngle(): Get vertical vertibrae value in 0-254 angle space
 *************************************************************************************/
byte getCurrentVerticalAngle(int i)
{
  return map(analogRead(VERT_POS_SENSOR[i]), vertLowCalibration[i], vertHighCalibration[i], 0, 254);
}

/**************************************************************************************
  printSensorValuesAndSetpoints(): Print sensor values and setpoints to the USB Serial
 *************************************************************************************/
void printSensorValuesAndSetpoints()
{
  USB_COM_PORT.print("\nSensors and Setpoints\n");
  USB_COM_PORT.print("Angles Values are 0-254: 255 = disabled\n");
  USB_COM_PORT.print("Sensors Values are 0-1023: -1 = disabled\n");

  USB_COM_PORT.print("\nHorizontal\n");
  USB_COM_PORT.print("\nHORZ_ANGLE_SETPOINT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_ANGLE_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << getCurrentHorizontalAngle(i) << "\t";
  }
  USB_COM_PORT.print("\nHORZ_ANGLE_DEADBAND:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleDeadband << "\t";
  }
  USB_COM_PORT << "\n";


  USB_COM_PORT.print("\nHORZ_SENSOR_SETPOINT:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << analogRead(HORZ_POS_SENSOR[i]) << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_DEADBAND:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorDeadbands[i] << "\t";
  }
  USB_COM_PORT << "\n";
  
  
  USB_COM_PORT.print("\nVertical\n");
  USB_COM_PORT.print("\nVERT_ANGLE_SETPOINT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << getCurrentVerticalAngle(i) << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_DEADBAND:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleDeadband << "\t";
  }
  USB_COM_PORT << "\n";

  USB_COM_PORT.print("\nVERT_SENSOR_SETPOINT:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << analogRead(VERT_POS_SENSOR[i]) << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_DEADBAND:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorDeadbands[i] << "\t";
  }
  USB_COM_PORT << "\n\n";

}


/**************************************************************************************
  printCalibrationValues(): Prints all calibration values.
 *************************************************************************************/
void printCalibrationValues()
{
  USB_COM_PORT.println("\nCalibration");
  USB_COM_PORT.print("HORZ_HIGH:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzHighCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_LOW:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzLowCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_RANGE: \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzHighCalibration[i] - horzLowCalibration[i] << "\t";
  }
  
  USB_COM_PORT.print("\n\nVERT_HIGH:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertHighCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_LOW:     \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertLowCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_RANGE: \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertHighCalibration[i] - vertLowCalibration[i] << "\t";
  }  
  USB_COM_PORT.print("\nVERT_STRAIGHT:\t");  
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertStraightArray[i] << "\t";
  }  
  USB_COM_PORT.println('\n');
}

/**************************************************************************************
  printBatteryVoltage(): Prints the battery voltage to the USB Serial port.
 *************************************************************************************/
void printBatteryVoltage()
{
  float batteryVolts = ((float)map(analogRead(BAT_LEVEL_24V),0,1023,0,25000))/1000;
  USB_COM_PORT << "The battery is at " << batteryVolts << "V\n";
}

/**************************************************************************************
  printHydraulicPressure(): Prints the pressure to the USB Serial port.
 *************************************************************************************/
void printHydraulicPressure()
{
  int readout = analogRead(PRESSURE_SENSOR);
  float voltage = (float)map(readout, 0, 1023, 0, 500) / 100;
  
  if (voltage < 0.45 || voltage > 4.55)
  {
    USB_COM_PORT << "ERROR: Pressure sensor should be between 0.5V and 4.5V. (" << voltage << "V)\n";
  }
  else
  {
    int hydraulicPressure = map(readout,102,921,0,2000); // 0.5-4.5V > 0-2000psi
    hydraulicPressure = (hydraulicPressure < 0) ? 0 : hydraulicPressure;    
    USB_COM_PORT << "The pressure is at " << hydraulicPressure << "psi (" << voltage << "V)\n";
  }
}


/**************************************************************************************
  saveVerticalPosition(): Saves the current vertical sensor positions
 *************************************************************************************/
void saveVerticalPosition()
{
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT.print("\nSaving current vertical position\n");
    
    // Get the current raw value
    vertStraightArray[i] = analogRead(VERT_POS_SENSOR[i]);
 
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
  setManualActuationDelay(): sets the actuation delay for manual mode actuation
 *************************************************************************************/
void setManualActuationDelay()
{
  StopMovement();
  while (USB_COM_PORT.available() < 1);
  char delaySetting = USB_COM_PORT.read();
  switch (delaySetting)
  {
    case 's':
      manualActuationDelay = 150;
      USB_COM_PORT.print("Actuation delay set to smallest time (150ms)\n");
      break;
    default:
    case 'm':
      manualActuationDelay = 200;
      USB_COM_PORT.print("Actuation delay set to medium time (200ms)\n");
      break;
    case 'l':
      manualActuationDelay = 300;
      USB_COM_PORT.print("Actuation delay set to largest time (300ms)\n");
      break;
  }
}

/**************************************************************************************
  manualVerticalActuatorMove(): Pulses to extend or contract the selected vertical actuator
 *************************************************************************************/
void manualVerticalActuatorMove(char dir)
{
  if (dir != 'e' && dir != 'c')
    return;
    
  StopMovement();

  // Setup the valve direction
  digitalWrite(VERT_ACTUATOR_CTRL[manualVertibraeSelect], LOW);
  if (dir == 'c')
  {
    digitalWrite(VERT_ACTUATOR_CTRL[manualVertibraeSelect], HIGH);
  }
  
  // Run actuation
  analogWrite(MOTOR_CONTROL, manualMotorSpeed);
  analogWrite(VERT_ACTUATOR[manualVertibraeSelect], 255);  
  delay(manualActuationDelay);
  StopMovement(); 
   
  USB_COM_PORT << "Vertibrae " << (manualVertibraeSelect + 1) << " " <<
    ((dir == 'e') ? "Vertical Extend" : "Vertical Contract") << " (" << manualActuationDelay << "ms)\n";  
}


/**************************************************************************************
  manualHorizontalActuatorMove(): Pulses to extend or contract the selected horizontal actuator
 *************************************************************************************/
void manualHorizontalActuatorMove(char dir)
{
  if (dir != 'e' && dir != 'c')
    return;
    
  StopMovement();

  // Setup the valve direction
  boolean even = PIDcontrollerHorizontal[manualVertibraeSelect].getEven();
  digitalWrite(HORZ_ACTUATOR_CTRL[manualVertibraeSelect], LOW);
  if (dir == 'c')
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[manualVertibraeSelect], HIGH);
  }
  
  // Run actuation
  analogWrite(MOTOR_CONTROL, manualMotorSpeed);
  analogWrite(HORZ_ACTUATOR[manualVertibraeSelect], 255);  
  delay(manualActuationDelay);
  StopMovement(); 
   
  USB_COM_PORT << "Vertibrae " << (manualVertibraeSelect + 1) << " " <<
    ((dir == 'e') ? "Hoirzontal Extend" : "Horizontal Contract") << " (" << manualActuationDelay << "ms)\n";  
}


/**************************************************************************************
  manualSetAllLEDs(): Turn all LEDs on or off
 *************************************************************************************/
void manualSetAllLEDs(boolean value)
{
  for (int i = 0; i < 5; i++)
  {
      digitalWrite(LED[i], value);
  }  
}

/**************************************************************************************
  manualSetModuleNumber(): manually set the module number from USB serial
 *************************************************************************************/
void manualSetModuleNumber()
{
  USB_COM_PORT << "Manually program myModuleNumber (0-9)...\n";
  while(USB_COM_PORT.available() < 1);
  int newNum = USB_COM_PORT.read() - 48;
  if (newNum < 0 || newNum > 6)
  {
    USB_COM_PORT << "Invalid\n";
    return;
  }
  USB_COM_PORT << "Now set to " << newNum << "\n";
  EEPROM.write(MY_MODULE_NUMBER_ADDRESS, newNum);
  setModuleNumber(newNum);
  return;    
}

/**************************************************************************************
  setModuleNumber(): Sets the module number and thus the even and odd vertibrae accordingly.
 *************************************************************************************/
void setModuleNumber(int value)
{
  myModuleNumber = value;
 
  // setup even/odd for even modules
  if (myModuleNumber % 2 == 0)
  {  
    PIDcontrollerVertical[0].setEven(false);
    PIDcontrollerVertical[1].setEven(true);
    PIDcontrollerVertical[2].setEven(false);
    PIDcontrollerVertical[3].setEven(true);
    PIDcontrollerVertical[4].setEven(false);
    
    PIDcontrollerHorizontal[0].setEven(false);
    PIDcontrollerHorizontal[1].setEven(true);
    PIDcontrollerHorizontal[2].setEven(false);
    PIDcontrollerHorizontal[3].setEven(true);
    PIDcontrollerHorizontal[4].setEven(false);
  }
  else
  {
    PIDcontrollerVertical[0].setEven(true);
    PIDcontrollerVertical[1].setEven(false);
    PIDcontrollerVertical[2].setEven(true);
    PIDcontrollerVertical[3].setEven(false);
    PIDcontrollerVertical[4].setEven(true);
    
    PIDcontrollerHorizontal[0].setEven(true);
    PIDcontrollerHorizontal[1].setEven(false);
    PIDcontrollerHorizontal[2].setEven(true);
    PIDcontrollerHorizontal[3].setEven(false);
    PIDcontrollerHorizontal[4].setEven(true);  
  }  
}


/**************************************************************************************
  runCommunicationTest(): Tests the communication between this module and a downstream 
                          module with the serial terminator. 
 *************************************************************************************/
boolean runCommunicationTest()
{
  // Figure out what module we're running the test to.
  USB_COM_PORT << "\nI am module " << myModuleNumber << ". What is the last module number?\n";
  while (USB_COM_PORT.available() == 0);

  int lastModuleNumber = USB_COM_PORT.read() - 48;
  if (lastModuleNumber < myModuleNumber || lastModuleNumber > 6)
  {
    USB_COM_PORT << "INVALID: " << lastModuleNumber << " is not a valid module number. Can only communicate downstream up to module 6.\n";
    return false;
  }
  USB_COM_PORT << "Running Communication Test: Module " << myModuleNumber << " to Module " << lastModuleNumber << "\n\n";
  USB_COM_PORT << "Send any key to start...\n\n";
  while (USB_COM_PORT.read() == -1);
  
  // Run 20 communication tests
  clearSerialBuffer(TAIL_SERIAL);
  int numberOfErrors = 0;
  
  for (int i = 1; i <= 20; ++i)
  {
    USB_COM_PORT << "Running test " << i << " of 20.\n";
    TAIL_SERIAL.write('t');
    delay(100);
    
    // We should recieve modules numbers followed by ASCII 't' aka ASCII 116
    long startTime = millis();
    int num = myModuleNumber + 1;
    boolean gotLastModuleT = false;
    boolean error = false;
    int numberOfModulesInTest = (lastModuleNumber - myModuleNumber);
    
    while(millis() - startTime < 150)
    {
      // Wait for data
      if (!TAIL_SERIAL.available())
      {
        continue;
      }
      byte data = TAIL_SERIAL.read();
      
      // If we've heard from every module look for ASCII charater 't'
      if (num > lastModuleNumber)
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
    if (!error && num <= lastModuleNumber)
    {
      USB_COM_PORT << "ERROR: Did not recieve data from all " << numberOfModulesInTest << " modules.\n";
      error = true;     
    }
    // If we didn't recieve 't' from the last module.
    if (!error && !gotLastModuleT)
    {
      USB_COM_PORT << "ERROR: Did not recieve 't' from the last module. Is the serial terminator plugged in?\n";
      error = true;
    }
    USB_COM_PORT << "\n";
    
    if(error) ++numberOfErrors;
  }
  
  // Give the result of the tests
  if (numberOfErrors > 0)
  {
    USB_COM_PORT << "RESULT: ERROR! Some communication tests failed.\n\n";
  }
  else
  {
    USB_COM_PORT << "RESULT: SUCCESS! All communication tests passed.\n\n";
  }
  delay(500);
  clearSerialBuffer(TAIL_SERIAL);
}

/**************************************************************************************
  manualTurnMotorOn(): Runs that motor at at one of three speed levels. manual mode must
                       be used to stop the motor.
 *************************************************************************************/

void manualTurnMotorOn()
{
  // Figure out what speed to run at
  while (USB_COM_PORT.available() == 0);
  
  char speedSelect = USB_COM_PORT.read();
  switch (speedSelect)
  {
    case '1':
      USB_COM_PORT << "Motor running at speed 80 (SEND 's' TO TURN OFF).\n";
      analogWrite(MOTOR_CONTROL, 80);      
      break;
    case '2':
      USB_COM_PORT << "Motor running at speed 160 (SEND 's' TO TURN OFF).\n";
      analogWrite(MOTOR_CONTROL, 160);  
      break;
    case '3':
      USB_COM_PORT << "Motor running at speed 240 (SEND 's' TO TURN OFF).\n";
      analogWrite(MOTOR_CONTROL, 240);      
      break;    
    default:
      USB_COM_PORT << "ERROR: " << speedSelect << " is not a valid speed selection.\n";
  }
}

/**************************************************************************************
  manualGoToSetpoints(): 
 *************************************************************************************/
void manualGoToSetpoints()
{
  USB_COM_PORT << "Moving... ";
  
  // reset timeout timers
  for (int i = 0; i < 5; ++i)
  {
    horzTimerArray[i] = millis();
    vertTimerArray[i] = millis();
  }
  
  // Do three seconds of movement
  unsigned long startTime = millis();
  while (millis() - startTime < 3000)
  {
    executePID(manualMotorSpeed, true, true);
    delay(pidLoopTime);
  }
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT << "Done.\n";
}

/**************************************************************************************
  setHorzAngleSetpoint(): 
 *************************************************************************************/
void setHorzAngleSetpoint(int actuator, byte angle)
{
    int i = actuator;
    int newHorzSetpoint = 0;
    if (angle == 255)
    {
      // 255 means this actuator is disabled.
      newHorzSetpoint = -1;
    }
    else
    {
      // Horizontal actuator is not disabled. Map to sensor space.
      newHorzSetpoint = map(angle, 0, 254, horzLowCalibration[i], horzHighCalibration[i]);
    }
     
    // If this is a new setpoint, reset the PID timout timer
    if (newHorzSetpoint != horzSensorSetpoints[i])
    {
      horzTimerArray[i] = millis();
      horzSensorSetpoints[i] = newHorzSetpoint;
      horzAngleSetpoints[i] = angle;
    }
}

/**************************************************************************************
  setVertAngleSetpoint(): 
 *************************************************************************************/
void setVertAngleSetpoint(int actuator, byte angle)
{
    int i = actuator;
    int newVertSetpoint = 0;
    if (angle == 255)
    {
      // 255 means this actuaor is disabled.
      newVertSetpoint = -1;
    }
    else if (angle == 127)
    {
      // We have a special setpoint for vertical angle 127 (straight)
      newVertSetpoint = vertStraightArray[i];
    }
    else if (PIDcontrollerVertical[i].getEven())
    {
      // Even vertical acutuators are mapped normally.
      newVertSetpoint = map(angle, 0, 254, vertLowCalibration[i], vertHighCalibration[i]);
    }
    else
    {
      // Since vertical sensors alternate sides, we need to invert the mapping from angle space to sensor space.
      newVertSetpoint = map(angle, 0, 254, vertHighCalibration[i], vertLowCalibration[i]);
    }  
    
    // If this is a new setpoint, copy it in and reset the timeout timer.
    if (newVertSetpoint != vertSensorSetpoints[i])
    {
      vertTimerArray[i] = millis();
      vertSensorSetpoints[i] = newVertSetpoint;
      vertAngleSetpoints[i] = angle;
    }  
}

/**************************************************************************************
  getNumberFromUsbComPort(): 
 *************************************************************************************/
boolean getNumberFromUsbComPort(int &number)
{
  // Wait for a character to arrive + extra time to allow more characters to arrive.
  while (USB_COM_PORT.available() == 0);
  delay(50);
  
  number = 0;
  
  // Add up the digits
  while (USB_COM_PORT.available())
  {
    char character = USB_COM_PORT.read();
    byte digit = character - 48;
    if (character == ' ') return true;
    if (digit > 9) return false;    
    number = number * 10 + digit;
  }
  return true;
}

/**************************************************************************************
  manualSetHorizontalAngle(): 
 *************************************************************************************/
 void manualSetHorizontalAngle()
 {
   int angle = 0;
   if (getNumberFromUsbComPort(angle))
   {
     if (angle < 0 || angle > 255)
     {
        USB_COM_PORT << "Invalid entry. (" << angle << ")\n";
        return; 
     }
     setHorzAngleSetpoint(manualVertibraeSelect, (byte)angle);  
     USB_COM_PORT << "Vertibrae " << manualVertibraeSelect + 1 << " Horizontal Angle set to " << angle << "\n";   
   }
   else
   {
      USB_COM_PORT << "Invalid entry. (" << angle << ")\n";     
   }
 }
 
 /**************************************************************************************
  manualSetVertAngleSetpoint():
 *************************************************************************************/
void manualSetVertAngleSetpoint()
{  
  int angle = 0;
  if (getNumberFromUsbComPort(angle))
  {
    if (angle < 0 || angle > 255)
    {
       USB_COM_PORT << "Invalid entry. (" << angle << ")\n";
       return; 
     }
     setVertAngleSetpoint(manualVertibraeSelect, (byte)angle);
     USB_COM_PORT << "Vertibrae " << manualVertibraeSelect + 1 << " Vertical Angle set to " << angle << "\n";
  }
  else
  {
    USB_COM_PORT << "Invalid entry. (" << angle << ")\n";     
  }   
}


/**************************************************************************************
  displayMenu(): Shows the command menu for manual control over usb serial
 *************************************************************************************/
void displayMenu()
{
    USB_COM_PORT.print("\nManual Control Mode\n");
    USB_COM_PORT.print("commands: 1-5 to select vertebrae\n");
    USB_COM_PORT.print("          k/l - horizontal actuation - extend/contract\n");
    USB_COM_PORT.print("          o/i - vertical actuation - extend/contract\n");
    USB_COM_PORT.print("          d* - adjust actuation delay where *=s(small), m(medium), l(large)\n");
    USB_COM_PORT.print("          r - save current vertical position as straight\n");
    USB_COM_PORT.print("          u - manually set myModuleNumber\n");
    USB_COM_PORT.print("          g - run communication test (between modules only)\n");
    USB_COM_PORT.print("          w* - set vertical setpoint\n");
    USB_COM_PORT.print("          z* - set horizontal setpoint\n");
    USB_COM_PORT.print("          j - go to setpoints\n"); 
    USB_COM_PORT.print("          c - calibrate horizontal\n");
    USB_COM_PORT.print("          v - calibrate verticals\n");
    USB_COM_PORT.print("          t - straighten horizontal\n");  
    USB_COM_PORT.print("          y - straighten verticals\n");  
    USB_COM_PORT.print("          a - print battery voltage\n");  
    USB_COM_PORT.print("          b - print calibration values\n");
    USB_COM_PORT.print("          f - print hydraulic pressure\n"); 
    USB_COM_PORT.print("          p - print sensor values and setpoints\n");
    USB_COM_PORT.print("          n/m - all leds on/off\n");
    USB_COM_PORT.print("          x* - turn motor on where *=1(slow), 2(medium), 3(fast)\n");
    USB_COM_PORT.print("          s - stop motor\n");  
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n\n");
}

/**************************************************************************************
  manualControl(): Allows for manual control of this module
 *************************************************************************************/
void manualControl()
{
  runPidLoop = false;
  displayMenu();
  
  boolean manual = true;
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      char manualCommand = USB_COM_PORT.read();      
      switch(manualCommand)
      {        
        case '1':
          manualVertibraeSelect = 0;
          USB_COM_PORT.print("Vertibrae 1 Selected\n");
          break;
        case '2':
          manualVertibraeSelect = 1;
          USB_COM_PORT.print("Vertibrae 2 Selected\n");
          break;
        case '3':
          manualVertibraeSelect = 2;
          USB_COM_PORT.print("Vertibrae 3 Selected\n");
          break;
        case '4':
          manualVertibraeSelect = 3;
          USB_COM_PORT.print("Vertibrae 4 Selected\n");
          break;
        case '5':
          manualVertibraeSelect = 4;
          USB_COM_PORT.print("Vertibrae 5 Selected\n");
          break;  
    
        case 'e':
          displayMenu();
          break;
        case 'w':
          manualSetVertAngleSetpoint();
          break;
        case 'z':
          manualSetHorizontalAngle();
          break;
        case 'j':
          manualGoToSetpoints();
          break;
        case 'c':
          calibrateHorizontal();
          break;
        case 'v':
          calibrateVertical();
          break;
        case 'b':
          printCalibrationValues();
          break;
        case 't':
          manualStraightenHorizontal();
          break;
        case 'y':
          manualStraightenVertical();
          break;
        case 'p':
          printSensorValuesAndSetpoints();
          break;
        case 'f':
          printHydraulicPressure();
          break;
        case 'r':
          saveVerticalPosition();
          break;
        case 'd':
          setManualActuationDelay();
          break;
        case 'l':
          manualHorizontalActuatorMove('c');
          break;
        case 'k':
          manualHorizontalActuatorMove('e');
          break;
        case 'i':
          manualVerticalActuatorMove('c');
          break;
        case 'o':
          manualVerticalActuatorMove('e');
          break;
        case 'n':
          manualSetAllLEDs(HIGH);
          break;
        case 'm':
          manualSetAllLEDs(LOW);
          break;
        case 'u':
          manualSetModuleNumber();
          break;
        case 'g':
          runCommunicationTest();
          break;
        case 'a':
          printBatteryVoltage();
          break;
        case 'x':
          manualTurnMotorOn();
          break;
        case 's':
          USB_COM_PORT.print("Stopped all movement\n");
          StopMovement();
          break;          
        case 'q':
          manual = false;
          break;
      }//end switch
    }//end if serial
  }  
  
  StopMovement();
  USB_COM_PORT.println("\nManual Control mode exited");
  runPidLoop = true;
  
  // Clear serial buffers to try and get back in sync
  clearSerialBuffer(HEAD_SERIAL);
  clearSerialBuffer(TAIL_SERIAL);  
}


/**************************************************************************************
  StopMovement(): Stops movement of all actuators.
 *************************************************************************************/
void StopMovement()
{
  // Stop any current motion
  for(int i=0; i < 5; i++)
  {
    analogWrite(VERT_ACTUATOR[i], 0);
    analogWrite(HORZ_ACTUATOR[i], 0);
  }
  analogWrite(MOTOR_CONTROL, 0);
  return;
}








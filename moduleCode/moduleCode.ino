/*

  moduleCode.ino - Controls 1 Titanaboa module of 5 vertebrae  
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA with a BoaShield to
  the control 5 vertical and 5 horizontal actuators of a 
  5 vertebrae module. Titanaboa moves by pushing a sequence of
  angles from head to tail at an interval initiated by an external 
  controller.
  
  Vertebrae angles are controlled by a hydrualic system in a PID
  loop with angular position sensors. Communication between the 
  modules is done over a serial daisy chain.  

*/

#include "EEPROM.h"
#include "titanoboa_pins.h"
#include "PIDcontrol.h"
#include "Lights.h"


// Defines and constants
#define HEAD_SERIAL Serial1             // Serial to the upstream module
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging

// Memory locations for items stored to EEPROM
#define MY_MODULE_NUMBER_ADDRESS  0
#define HORIZONTAL_CALIBRATION_ADDRESS  2
#define VERTICAL_CALIBRATION_ADDRESS  (HORIZONTAL_CALIBRATION_ADDRESS + 25)
#define VERTICAL_STRAIGHT_ADDRESS  (VERTICAL_CALIBRATION_ADDRESS + 25)

#define CALIBRATE_MOTOR_SPEED  150
#define JAW_MOTOR_SPEED        90

const int VERT_DEAD_ZONE = 20;          // Vertical safety range


// Static variables
boolean even = false;                   // Unused, will be set in EEPROM after calibration

char horzAngleArray[] = {               // Current horizontal and vertical position of 
  '3','3','3','3','3'};                 // the actuators.
char vertAngleArray[] = {               // 0 = Bent Left, 1 = Bent Right  <-- TODO:confirm
  '3','3','3','3','3'};                 // 2 = Straight,  3 = Disabled

// Array of angles we recorded as straight for the verticals
int vertStraightArray[] = {0, 0, 0, 0, 0};


unsigned long horzTimerArray[] = {      // Timer arrays used to timeout when waiting for 
  0,0,0,0,0};                           // an actuator to reach its set point
unsigned long vertTimerArray[] = {
  0,0,0,0,0};  

// Calibration values. The values of the horizontal position sensors at the hard limits of actuation.
// Assume a linear sensor characteristic between these two points.
int highRange[] = {1023, 1023, 1023, 1023, 1023};
int lowRange[] = {0, 0, 0, 0, 0};

// Calibration values. The values of the vertical position sensors at the hard limits of actuation.
// Assume a linear sensor characteristic between these two points.
int vertHighRange[] = {1023, 1023, 1023, 1023, 1023};
int vertLowRange[]  = {0, 0, 0, 0, 0};
              
// Settings
byte settings[125];
      
byte myModuleNumber;                    // My position in the module chain (1,2,3...)
byte endModuleNumber;                   // ? TODO:Not sure why we would need this
byte killSwitch = true;                 // True if no movement should be done.
int motorSpeed = 200;                   // Analog output for pump speed when turned on

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

// lights object for cotrolling lights
Lights myLights = Lights();  //default is off


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
  endModuleNumber = EEPROM.read(1);  
  for(int i=0;i<5;i++)
  { 
    highRange[i] = EEPROM.read(i*5+2) + (EEPROM.read(i*5+3) << 8);
    lowRange[i] = EEPROM.read(i*5+4) + (EEPROM.read(i*5+5) << 8);
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

  // Initialize the horizontal angle array based on the current
  // position of the pistons.
  for(int i=0;i<5;i++)
  {
    int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);
    
    if(currentAngle < 100)
      horzAngleArray[i] = '0';
    if(currentAngle >= 100 && currentAngle <= 154)
      horzAngleArray[i] = '2';
    if(currentAngle > 154)
      horzAngleArray[i] = '1';
  }

  // Print out the loaded cabibration and angle array.
  USB_COM_PORT.print("\nHi I'm Titanoboa, MODULE #: ");
  USB_COM_PORT.println(myModuleNumber, DEC);

  USB_COM_PORT.print("\nCurrent voltage reading: ");
  USB_COM_PORT.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,25000));

  USB_COM_PORT.println("> Loaded Calibration and Initialized Horizontal Angle Array");
  USB_COM_PORT.print("HIGH: ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(highRange[i]);
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.print("\nLOW: ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(lowRange[i]);
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.print("\nEVEN?: "); 
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(PIDcontrollerHorizontal[i].getEven() ? 'T' : 'F');
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.print("\nANGLE_ARRAY: ");  
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(horzAngleArray[i]);
    USB_COM_PORT.print('\t');
  }
  USB_COM_PORT.println('\n');

  // Print out saved vertical straight angle array
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT.print("Saved vertical straight position for vertebrae ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(" is ");
    USB_COM_PORT.println(vertStraightArray[i]);
  }

  // Print out even/odd setting for verticals
  USB_COM_PORT.print("\nEVEN?: ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(PIDcontrollerHorizontal[i].getEven() ? 'T' : 'F');
    USB_COM_PORT.print('\t');
  }
  delay(1000);
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
    //letters in use: c, h, g, l, k
    char command = HEAD_SERIAL.read();
    switch (command)
    {
      case 's':
        processNewSettingsAndSetpoints();
        break;
      
      case 'c':
        processCalibrateCommand();
        break;
      
      case 'h':
        processLongMotorPulseCommand();
        break;
        
      case 'g':
        processShortMotorPulseCommand();
        break;

      default:
        USB_COM_PORT.print("Invalid command from upstream serial = ");
        USB_COM_PORT.println(command);
        HEAD_SERIAL.flush();
        
    }
  }
  while (TAIL_SERIAL.available() > 0)
  {
    HEAD_SERIAL.write(TAIL_SERIAL.read());
  }

}//end loop()

/************************************************************************************
  processNewSettingsAndSetpoints(): Recieves new settings and setpoints from the head
 ***********************************************************************************/

void processNewSettingsAndSetpoints()
{
  // Get data array from upstream. Store in an array.
  for (int i = 0; i < 125; ++i)
  {
    settings[i] = HEAD_SERIAL.read();
  }
  
  // Send data array downstream.
  TAIL_SERIAL.write('s');
  for (int i = 0; i < 125; ++i)
  {
    TAIL_SERIAL.write(settings[i]);
  }
  
  // Check the kill switch [Setting bit 70.0]
  killSwitch = (settings[70] && 0b00000001) > 0;
  if (killSwitch == true)
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
  
  // Copy new setpoints [Setting bytes 0 to 59]
  for (int i = 0; i < 5; ++i)
  {
    horzAngleArray[i] = (char)settings[myModuleNumber * 5 + i];
    vertAngleArray[i] = (char)settings[myModuleNumber * 5 + i + 30];    
  }
}
 
 /************************************************************************************
  processRunPIDCommand(): Runs the PID loop on each actuator
 ***********************************************************************************/
 
void processRunPIDCommand()
{
 
}
 
 /************************************************************************************
  processDiagnosticsCommand(): Responds to the head's request to get diagnsotic info.
 ***********************************************************************************/

void processDiagnosticsCommand()
{
 
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
  move(): Modulates the values to achieve position set points (Runs the PID)
 ***********************************************************************************/

const int MAX_WAIT_TIME = 3000;  // Time to try moving actuator before giving up
const int MAX_WAIT_TIME_VERT = 1000;
const int DEAD_ZONE = 10;        // Safety range to not move
int count = 0;

void move()
{
  //set if any segment moved, so we can turn off motor when setpoint reached
  boolean moved = false;
  int goal;
  int currentAngle;

  for (int i=0; i < 5; i++)
  {
    ////// HORIZONTAL ////////
    
    if (horzAngleArray[i]=='0')
    {
      goal = 10;
    }
    else if (horzAngleArray[i]=='1')
    {
      goal = 245;
    }
    else if (horzAngleArray[i]=='2')
    {
      goal = 127;
    }
    else
    {
      analogWrite(HORZ_ACTUATOR[i],0);
      continue;
    }

    PIDcontrollerHorizontal[i].setSetPoint(map(goal,0,255,lowRange[i],highRange[i]));
    currentAngle = map(analogRead(HORZ_POS_SENSOR[i]), lowRange[i], highRange[i], 0, 255);
    
    // If we are have not reached the deadzone and haven't timed out
    if ((abs(currentAngle-goal)>DEAD_ZONE) &&
       (millis()-horzTimerArray[i]) < MAX_WAIT_TIME)
    {
      // Update the acutator PID output
      analogWrite(MOTOR_CONTROL, motorSpeed);
      PIDcontrollerHorizontal[i].updateOutput();
      moved = true;
    }
    // We are in the deadzone or we took too long to get there    
    else
    {
      analogWrite(HORZ_ACTUATOR[i],0);
    }
    
    ////// VERITICAL ////////
    
    // Currently the vertical actuators only know how to straighten
    if (vertAngleArray[i] == '2')
    {
      PIDcontrollerVertical[i].setSetPoint(vertStraightArray[i]);
      currentAngle = analogRead(VERT_POS_SENSOR[i]);
      
      // If we are have not reached the deadzone and haven't timed out
      if ((abs(currentAngle - vertStraightArray[i]) > DEAD_ZONE) &&
          (millis() - vertTimerArray[i]) < MAX_WAIT_TIME)
      {
        // Update the acutator PID output        
        analogWrite(MOTOR_CONTROL, motorSpeed);
        PIDcontrollerVertical[i].updateOutput();
        moved = true;
      }
      // We are in the deadzone or we took too long to get there
      else
      {
        analogWrite(VERT_ACTUATOR[i], 0);
      }
    }
    
    // If were not in straightening mode, turn actuator off.
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


/***********************************************************************************
  processCalibrateCommand(): run horizontal and vertical calibration
 ***********************************************************************************/
void processCalibrateCommand()
{
    // Determine my modules number
    USB_COM_PORT.println("Calibrate command received");
    while (HEAD_SERIAL.available() < 1);
    myModuleNumber = HEAD_SERIAL.read();

    // Tell next module to also calibrate
    TAIL_SERIAL.write('c');
    TAIL_SERIAL.write(myModuleNumber + 1);

    //calibrate horizontal position and reset everything
    calibrateHorizontal();
    HEAD_SERIAL.flush();
    TAIL_SERIAL.flush();

    delay(100);
    
    //calibrate vertical position and reset everything
    //calibrateVertical();
    HEAD_SERIAL.flush();
    TAIL_SERIAL.flush();

    delay(100);

} //end processCalibrateCommand()


/**************************************************************************************
  calibrateHorizontal(): Records horizontal sensor values at the hard limits of each 
                         vertebrae. 
                         Saves to EEPROM.
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
    highRange[i] = analogRead(HORZ_POS_SENSOR[i]);
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
    lowRange[i] = analogRead(HORZ_POS_SENSOR[i]);
  }

  // Adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++)
  {
    if(highRange[i]>lowRange[i])
    {
      PIDcontrollerHorizontal[i].setEven(true);
      horzAngleArray[i] = '1';
    }
    else
    {
      int temp = highRange[i];
      highRange[i] = lowRange[i];
      lowRange[i] = temp;
      PIDcontrollerHorizontal[i].setEven(false);
      horzAngleArray[i] = '0';
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  EEPROM.write(0,myModuleNumber);
  EEPROM.write(1,endModuleNumber);
  for(int i=0;i<5;i++){ 
    EEPROM.write(i*5+2,lowByte(highRange[i]));
    EEPROM.write(i*5+3,highByte(highRange[i]));
    EEPROM.write(i*5+4,lowByte(lowRange[i]));
    EEPROM.write(i*5+5,highByte(lowRange[i]));
    EEPROM.write(i*5+6,PIDcontrollerHorizontal[i].getEven());
  }

  straightenHorizontal();
   
  // Print out calibration
  USB_COM_PORT.println("");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(highRange[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("HIGH ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(lowRange[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("RANGE ");
  for(int i=0;i<5;i++){
    USB_COM_PORT.print(highRange[i] - lowRange[i]);
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
    vertHighRange[i] = analogRead(VERT_POS_SENSOR[i]);
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
    vertLowRange[i] = analogRead(VERT_POS_SENSOR[i]);
  }
  
  // Adjust high and low range and account for alternating actuators
  /*for(int i=0;i<5;i++)
  {
    if(vertHighRange[i]>vertLowRange[i])
    {
      PIDcontrollerVertical[i].setEven(false);
      vertAngleArray[i] = '1';
    }
    else
    {
      int temp = vertHighRange[i];
      vertHighRange[i] = vertLowRange[i];
      vertLowRange[i] = temp;
      PIDcontrollerVertical[i].setEven(true);
      vertAngleArray[i] = '0';
    }
  }*/
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  //EEPROM.write(0,myModuleNumber);
  //EEPROM.write(1,endModuleNumber);
  /*for(int i=0;i<5;i++){ 
    EEPROM.write(i*5+2, lowByte(vertHighRange[i]));
    EEPROM.write(i*5+3, highByte(vertHighRange[i]));
    EEPROM.write(i*5+4, lowByte(vertLowRange[i]));
    EEPROM.write(i*5+5, highByte(vertLowRange[i]));
    EEPROM.write(i*5+6, PIDcontrollerVertical[i].getEven());
  }*/

  straightenVertical();
   
  // Print out calibration
  USB_COM_PORT.println("");
  for (int i=0; i<5; i++)
  {
    USB_COM_PORT.print(vertHighRange[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("HIGH ");
  for (int i=0; i<5; i++)
  {
    USB_COM_PORT.print(vertLowRange[i]);
    USB_COM_PORT.print(' ');
  }
  USB_COM_PORT.println("LOW ");
  for(int i=0; i<5; i++)
  {
    USB_COM_PORT.print(vertHighRange[i] - vertLowRange[i]);
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
  straightenHorizontal(): Makes Titanaboa straight as an arrow.
 *************************************************************************************/
void straightenHorizontal()
{
  // Move till horzAngleArray is matched
  boolean inPosition = false;

  int goal = 127;
  for(int i=0;i<5;i++)
  {
    PIDcontrollerHorizontal[i].setSetPoint(map(goal,0,255,lowRange[i],highRange[i]));
  }

  // Time limit so if stuck, goes to next
  unsigned long startTime = millis();
  while(inPosition == false && (millis()-startTime) < MAX_WAIT_TIME){
    inPosition = true;

    for(int i=0;i<5;i++){
      int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

      if(abs(currentAngle-goal)>DEAD_ZONE){
        analogWrite(MOTOR_CONTROL, CALIBRATE_MOTOR_SPEED);
        PIDcontrollerHorizontal[i].updateOutput();
        inPosition=false;
      }
      else{
        analogWrite(HORZ_ACTUATOR[i],0);
      }
    }
  }

  //turn off motor
  analogWrite(MOTOR_CONTROL, 0);
  //turn off all actuators
  //undefine angles
  for(int i=0;i<5;i++){
    analogWrite(HORZ_ACTUATOR[i],0);
    horzAngleArray[i] = '2';
  }
} //end straightenHorizontal()


/**************************************************************************************
  straightenVertical(): Makes Titanaboa straight as an arrow...vertically
 *************************************************************************************/
void straightenVertical()
{
  // Move till vertAngleArray is matched???
  boolean inPosition = false;
  
  USB_COM_PORT.println("\nstraightening vertical");
  for (int i=0; i<5; i++)
  {
    PIDcontrollerVertical[i].setSetPoint(vertStraightArray[i]);
    USB_COM_PORT.print("current vertical position for vertebrae ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(" is ");
    USB_COM_PORT.print(analogRead(VERT_POS_SENSOR[i]));
    USB_COM_PORT.print(", target is ");
    USB_COM_PORT.println(vertStraightArray[i]);
  }

  // Time limit so if stuck, goes to next
  unsigned long startTime = millis();
  while (inPosition == false && (millis()-startTime) < MAX_WAIT_TIME_VERT)
  {
    inPosition = true;

    for (int i=0; i<5; i++)
    {
      int currentAngle = analogRead(VERT_POS_SENSOR[i]);

      if (abs(currentAngle - vertStraightArray[i]) > DEAD_ZONE)
      {
        analogWrite(MOTOR_CONTROL, CALIBRATE_MOTOR_SPEED);
        PIDcontrollerVertical[i].updateOutput();
        inPosition = false;
      }
      else
      {
        analogWrite(VERT_ACTUATOR[i], 0);
      }
    }
  }

  //turn off motor
  analogWrite(MOTOR_CONTROL, 0);
  //turn off all actuators
  //undefine angles
  for (int i=0; i<5; i++)
  {
    analogWrite(VERT_ACTUATOR[i],0);
    vertAngleArray[i] = '2';
  }
  
  for (int i=0; i<5; i++)
  {
    USB_COM_PORT.print("current vertical position for vertebrae ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(" is ");
    USB_COM_PORT.println(analogRead(VERT_POS_SENSOR[i]));
  }
} //end straightenVertical()


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
    vertLowRange[i] = vertStraightArray[i] - 100;
    vertHighRange[i] = vertStraightArray[i] + 100;

    USB_COM_PORT.print("Vertical sensor ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(": current position ");
    USB_COM_PORT.print(vertStraightArray[i]);
    USB_COM_PORT.print(", low limit ");
    USB_COM_PORT.print(vertLowRange[i]);
    USB_COM_PORT.print(", high limit ");
    USB_COM_PORT.println(vertHighRange[i]);
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

    displayMenu();

  while(manual == true)
  {
    if(Serial.available() > 0){
      byteIn = USB_COM_PORT.read();
      
      switch(byteIn)
      {
        case 'c':
          USB_COM_PORT.print("\nRunning calibration...\n");
          calibrateHorizontal();
          break;
        case 'p':
          readSensors();
          break;
        case 'r':
          USB_COM_PORT.print("\nSaving current vertical position\n");
          saveVerticalPosition();
          break;
        case 'v':
          // only straighten verticals if there is a stored vertical position
          if ((vertStraightArray[0] != 0) &&
              (vertStraightArray[1] != 0) &&
              (vertStraightArray[2] != 0) &&
              (vertStraightArray[3] != 0) &&
              (vertStraightArray[4] != 0))
            {
              USB_COM_PORT.print("\nStraightening verticals\n");
              straightenVertical();
            }
            else
            {
              USB_COM_PORT.print("\nUnable to straighten as there is no stored data\n");
            }
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
    USB_COM_PORT.print("          v - straighten verticals\n");
    USB_COM_PORT.print("          s - stop motor\n");
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








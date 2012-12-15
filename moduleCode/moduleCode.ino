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
#define CALIBRATE_MOTOR_SPEED  200

const int VERT_DEAD_ZONE = 20;          // Vertical safety range


// Static variables
boolean even = false;                   // Unused, will be set in EEPROM after calibration

char horzAngleArray[] = {               // Current horizontal and vertical position of 
  '3','3','3','3','3'};                 // the actuators.
char vertAngleArray[] = {               // 0 = Bent Left, 1 = Bent Right  <-- TODO:confirm
  '3','3','3','3','3'};                 // 2 = Straight,  3 = Undefined

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
  loop(): Checks for serial messages and runs the PID controllers in an endless loop.
 ***********************************************************************************/
 
char headAngle[] = {                    // Angles recieved from the upsteam module
  '3','3'  };                           // module are temporarily stored here. 
char tailAngle[] = {                    // ? TODO:not sure why we need angles from the 
  '3','3'  };                           // downstream module
 
void loop()
{
  headAngle[0] = '3';
  headAngle[1] = '3';

  char message;

  // check for command from USB serial to enter manual control
  // NOTE: this is a quick hack for NightQuest so that we can adjust actuators quickly if they get out of wack
  //       this only puts one module in manual control, never operate titanoboa like this!!!
  //       in future manual control mode should be selected from the joystick and all modules enter manual control
  //       mode and return to normal operation together
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

  // check for messages from upstream port
  if (HEAD_SERIAL.available() > 0)
  {
    //letters in use: m, s, L, v, c, h, g, l, M, k
    switch (HEAD_SERIAL.read())
    {
    //update motor speed
    case 'm':
        USB_COM_PORT.print("new motor speed received: ");
        while (HEAD_SERIAL.available() < 1);
        motorSpeed = HEAD_SERIAL.read();
        TAIL_SERIAL.write('m');
        TAIL_SERIAL.write((char)motorSpeed);
        USB_COM_PORT.print("new motor speed received: ");
        USB_COM_PORT.println(motorSpeed);
        break;

    case 's':
      //setpionts to come
      headAngle[0] = HEAD_SERIAL.read();  //get horizontal setpoint angle
      
      // If we recieve new setpoints, disable kill switch
      killSwitch = false;
      break;
    
    case 'L':
      //Light Propagation
      USB_COM_PORT.println("L");
      while(HEAD_SERIAL.available() < 1);
      message = HEAD_SERIAL.read();
      if(message == 'm'){
        //change mode
        while(HEAD_SERIAL.available() < 1);
        message = HEAD_SERIAL.read();
        myLights.changeMode(message-'0');
        TAIL_SERIAL.print("Lm");
        TAIL_SERIAL.print(message);
      }
      else{
        if(message == '1'){
          message = myLights.push(true);
        }
        else{
          message = myLights.push(false);
        }
        TAIL_SERIAL.write('L');
        TAIL_SERIAL.write(message ? '1':'0');
      }
      break;

    case 'v':
      // display current voltage level
      USB_COM_PORT.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,25000));
      break;
     
    case 'c':
      //calibration
      USB_COM_PORT.println("Calibrate command received");
      while (HEAD_SERIAL.available() < 1);
      myModuleNumber = HEAD_SERIAL.read();

      // Tell next module to also calibrate
      TAIL_SERIAL.write('c');
      TAIL_SERIAL.write(myModuleNumber + 1);

      calibrate();
      break;
      
    case 'h':
      //for moving head, turns on and off motor
      while(HEAD_SERIAL.available()<1)
      {
        delay(1);
      }

      message = HEAD_SERIAL.read();
      if(message == '5')
      {
        digitalWrite(HORZ_ACTUATOR_CTRL[0],LOW);
      }
      else if(message == '6')
      {
        digitalWrite(HORZ_ACTUATOR_CTRL[0],HIGH);
      }
      
      if(message == '5' || message == '6')
      {
        for(int i=0;i<=255;i++)
        {
          analogWrite(HORZ_ACTUATOR[0],i);
          delay(2);
        }
        for(int i=255;i>=0;i--)
        {
          analogWrite(HORZ_ACTUATOR[0],i);
          delay(2);
        }
      }

      analogWrite(MOTOR_CONTROL, motorSpeed);
      delay(1000);
      analogWrite(MOTOR_CONTROL,0);
      break;
      
    case 'g':
      //manual control of head actuators, need to turn motor on and off
      USB_COM_PORT.print("turning motor on\n");
      analogWrite(MOTOR_CONTROL, motorSpeed);
      delay(200);  //for now just run for 200ms
      analogWrite(MOTOR_CONTROL, 0);
      USB_COM_PORT.print("turning motor off\n");
      break;

    case 'l':
      // straighten verticals
      straightenVertical();
      TAIL_SERIAL.print("l");
      ready();
      break;
      
    case 'M':
      // manual actuator control
      manualControl();
      break;
      
    case 'k':
      // Kill Switch
      killSwitch = true;
      for(int i=0; i < 5; i++)
      {
        analogWrite(VERT_ACTUATOR[i], 0);
        analogWrite(HORZ_ACTUATOR[i], 0);
      }

      //ensure the motor is turned off and pass the message down
      analogWrite(MOTOR_CONTROL, 0);
      TAIL_SERIAL.write('k');
      break;
    }

    //flush everything and get ready for next loop
    HEAD_SERIAL.flush();
  }

  if(TAIL_SERIAL.available()>0)
  {
    switch(TAIL_SERIAL.read())
    {
    case 'r':
      while(TAIL_SERIAL.available()<1)
      {
        delay(1);
      }
      HEAD_SERIAL.write('r');
      HEAD_SERIAL.write(TAIL_SERIAL.read());

      break;
    }
    TAIL_SERIAL.flush();
  }


  // Move the snake into position, if the kill switch it disabled
  if(!killSwitch)
  {
    move();
  }

  //update lights
  myLights.update();

  //head angles not '0', '1' or '2' are not recognized
  if(headAngle[0] != '0' &&
    headAngle[0] != '1' &&
    headAngle[0] != '2' &&
    headAngle[1] != '0' &&
    headAngle[1] != '1' &&
    headAngle[1] != '2')
  {
    return;
  }

  //read new angle and propagate it down
  propagate();

  TAIL_SERIAL.write('s');
  //send the next angle down
  TAIL_SERIAL.write(tailAngle[0]);

  //ready to go one
  ready();

}//end loop()


/***********************************************************************************
  ready(): TODO: Why do we need this?
 ***********************************************************************************/
void ready()
{
  HEAD_SERIAL.write('r');
  HEAD_SERIAL.write(myModuleNumber);
}


/************************************************************************************
  propagate(): Properly moves angles downstream and out towards the following module
 ***********************************************************************************/
void propagate()
{
  tailAngle[0] = horzAngleArray[4];
  tailAngle[1] = vertAngleArray[4];

  for(int i=4;i>0;i--)
  {
    if(horzAngleArray[i] != horzAngleArray[i-1])
    {
      horzTimerArray[i] = millis();
    }
    horzAngleArray[i] = horzAngleArray[i-1];

    if(vertAngleArray[i] != vertAngleArray[i-1])
    {
      vertTimerArray[i] = millis();
    }
    vertAngleArray[i] = vertAngleArray[i-1];
  }

  if(horzAngleArray[0] != headAngle[0])
  {
    horzTimerArray[0] = millis();
  }
  horzAngleArray[0] = headAngle[0];

  if(vertAngleArray[0] != headAngle[1])
  {
    vertTimerArray[0] = millis();
  }
  vertAngleArray[0] = headAngle[1];

  USB_COM_PORT.print('\n');
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT.print(horzAngleArray[i]);
  }
  USB_COM_PORT.println(' horz\n');

  for(int i=0;i<5;i++)
  {
    USB_COM_PORT.print(vertAngleArray[i]);
  }
  USB_COM_PORT.println(' vert');

}//end propagate()


/************************************************************************************
  move(): Modulates the values to achieve position set points (Runs the PID)
 ***********************************************************************************/

const int MAX_WAIT_TIME = 3000;  // Time to try moving actuator before giving up
const int MAX_WAIT_TIME_VERT = 1000;
const int DEAD_ZONE = 10;        // Safty range to not move 
int count = 0;

void move()
{
  //to see if any in segment moved
  //so we can turn off motor when it's reached setpoint
  boolean moved = false;

  int goal;

  for (int i=0; i < 5; i++)
  {
    //horizontal
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

    int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

    if ((abs(currentAngle-goal)>DEAD_ZONE) &&
       (millis()-horzTimerArray[i]) < MAX_WAIT_TIME)
    {
      analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
      PIDcontrollerHorizontal[i].updateOutput();
      moved = true;
    }
    else
    {
      analogWrite(HORZ_ACTUATOR[i],0);
    }

    //vertical
    if((millis()-vertTimerArray[i]) < MAX_WAIT_TIME)
    {
      analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
      if(vertAngleArray[i] == '0')
      {
        digitalWrite(VERT_ACTUATOR_CTRL[i],LOW);
        analogWrite(VERT_ACTUATOR[i],255);
        moved = true;
      }
      else if(vertAngleArray[i] == '1')
      {
        digitalWrite(VERT_ACTUATOR_CTRL[i],HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
        moved = true;
      }
      else if(vertAngleArray[i] == '2')
      {
        //do nothing
      }
    }
    else
    {
      analogWrite(VERT_ACTUATOR[i],0);
    }
  }
  if(moved == false)
  {
    analogWrite(MOTOR_CONTROL, 0);
  }
}


/***********************************************************************************
  calibrate(): run horizontal and vertical calibration
 ***********************************************************************************/
void calibrate()
{
    //calibrate horizontal position and reset everything
    calibrateHorizontal();
    HEAD_SERIAL.flush();
    TAIL_SERIAL.flush();

    delay(100);
    ready();
    
    //calibrate vertical position and reset everything
    //calibrateVertical();
    HEAD_SERIAL.flush();
    TAIL_SERIAL.flush();

    delay(100);
    ready();
      
} //end calibrate()


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

  straighten();
   
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
  straighten(): Makes Titanaboa straight as an arrow.
 *************************************************************************************/
void straighten()
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
} //end straighten()


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
          calibrate();
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
  USB_COM_PORT.print("\nManual Control mode exited");
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
    USB_COM_PORT.print("          c - calibrate\n");
    USB_COM_PORT.print("          p - print raw values of position sensors\n");
    USB_COM_PORT.print("          r - save current vertical position as straight\n");
    USB_COM_PORT.print("          v - straighten verticals\n");
    USB_COM_PORT.print("          s - stop motor\n");
    USB_COM_PORT.print("          e - menu");
    USB_COM_PORT.print("          q - quit\n");
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






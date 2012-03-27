/*

  moduleCode.h - Controls 1 Titanaboa module of 5 vertebrae  
  
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
#include "boashield_pins.h"
#include "PIDcontrol.h"

boolean even = false;                   // Unused, will be set in EEPROM after calibration
const int MOTOR_SPEED = 130;            // Analog output for pump speed when turned on

char horzAngleArray[] = {               // Current horizontal and vertical position of 
  '3','3','3','3','3'};                 // the actuators.
char vertAngleArray[] = {               // 0 = Bent Left, 1 = Bent Right  <-- TODO:confirm
  '3','3','3','3','3'};                 // 2 = Straight,  3 = Undefined

unsigned long horzTimerArray[] = {      // Timer arrays used to timeout when waiting for 
  0,0,0,0,0};                           // an actuator to reach its set point
unsigned long vertTimerArray[] = {
  0,0,0,0,0};  

int highRange[] = {                     // Calibration values. The values of the position
  1023,1023,1023,1023,1023};            // sensors at the hard limits of actuation. Assume
int lowRange[] = {                      // a linear sensor characteristic between these
  0,0,0,0,0};                           // two points.
                                        
char myModuleNumber;                    // My position in the module chain (1,2,3...)
char endModuleNumber;                   // ? TODO:Not sure why we would need this

#define HEAD_SERIAL Serial1             // Serial to the upstream module
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define USB_COM_PORT Serial             // Serial for debugging

// PID Controllers for the horizontal actuators
PIDcontrol PIDcontroller[] = {
  PIDcontrol(HORZ_POS_SENSOR[0], HORZ_ACTUATOR_CTRL[0], HORZ_ACTUATOR[0], even),
  PIDcontrol(HORZ_POS_SENSOR[1], HORZ_ACTUATOR_CTRL[1], HORZ_ACTUATOR[1], !even),
  PIDcontrol(HORZ_POS_SENSOR[2], HORZ_ACTUATOR_CTRL[2], HORZ_ACTUATOR[2], even),
  PIDcontrol(HORZ_POS_SENSOR[3], HORZ_ACTUATOR_CTRL[3], HORZ_ACTUATOR[3], !even),
  PIDcontrol(HORZ_POS_SENSOR[4], HORZ_ACTUATOR_CTRL[4], HORZ_ACTUATOR[4], even),
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

  // Print out a hello message
  Serial.println("Hi I'm Titanoboa");

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
  }

  // Set the constants of the PID controllers
  for(int i=0;i<5;i++)
  {
    PIDcontroller[i].setConstants(50,0,0);
  }

  // Load previous calibration from EEPROM
  myModuleNumber = EEPROM.read(0);
  endModuleNumber = EEPROM.read(1);  
  for(int i=0;i<5;i++)
  { 
    highRange[i] = EEPROM.read(i*5+2) + (EEPROM.read(i*5+3) << 8);
    lowRange[i] = EEPROM.read(i*5+4) + (EEPROM.read(i*5+5) << 8);
    PIDcontroller[i].setEven(EEPROM.read(i*5+6));
  }
  
  // Initialize the angle array based on the current
  // position of the pistons.
  for(int i=0;i<5;i++)
  {
    int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);
    
    if(currentAngle < 100)
      horzAngleArray[i]=='0';
    if(currentAngle >= 100 && currentAngle <= 154)
      horzAngleArray[i]=='2';
    if(currentAngle > 154)
      horzAngleArray[i]=='1';
  } 

  // Print out the loaded cabibration and angle array.
  Serial.println("> Loaded Calibration and Initialized Angle Array");
  Serial.print("MODULE #: ");
  Serial.println(myModuleNumber, DEC);
  Serial.print("HIGH: ");
  for(int i=0;i<5;i++){
    Serial.print(highRange[i]);
    Serial.print('\t');
  }
  Serial.print("\nLOW: ");
  for(int i=0;i<5;i++){
    Serial.print(lowRange[i]);
    Serial.print('\t');
  }
  Serial.print("\nEVEN?: "); 
  for(int i=0;i<5;i++){
    Serial.print(PIDcontroller[i].getEven() ? 'T' : 'F');
    Serial.print('\t');
  }
  Serial.print("\nANGLE_ARRAY: ");  
  for(int i=0;i<5;i++){
    Serial.print(horzAngleArray[i]);
    Serial.print('\t');
  }
  Serial.println('\n');  
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

  if (HEAD_SERIAL.available() > 0)
  {

    switch (HEAD_SERIAL.read())
    {
    case 's':
      //setpionts to come
      while(HEAD_SERIAL.available() < 2)
      {
        delay(1);
      }
      headAngle[0] = HEAD_SERIAL.read();
      headAngle[1] = HEAD_SERIAL.read();
      break;

    case 'v':
      Serial.println(map(analogRead(BAT_LEVEL_24V),0,1023,0,25000));
      break;
     
    case 'c':
      //calabration
      //my module number and end module number to come
      //brain is 0, first moving segment is 1 etc
      while(HEAD_SERIAL.available()<2)
      {
        delay(1);
      }

      //tell next module to also calibrate
      TAIL_SERIAL.write('c');
      myModuleNumber = HEAD_SERIAL.read();
      endModuleNumber = HEAD_SERIAL.read();

      even = (myModuleNumber%2)==0;

      TAIL_SERIAL.write(myModuleNumber+1);
      TAIL_SERIAL.write(endModuleNumber);

      //calibrate and reset everything
      calibrate();
      HEAD_SERIAL.flush();
      TAIL_SERIAL.flush();

      delay(100);
      ready();
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

      analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
      delay(1000);
      analogWrite(MOTOR_CONTROL,0);
      break;

    case 'l':
      TAIL_SERIAL.print("l");
      straighten();
      ready();
      break;
      
    case 'A':
      // manual actuator control
      manualControl();
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


  //move the snake into position
  move();

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

  //read new angle and propergate it down
  propergate();

  TAIL_SERIAL.write('s');
  //send the next angle down
  TAIL_SERIAL.write(tailAngle[0]);
  TAIL_SERIAL.write(tailAngle[1]);


  //ready to go one
  ready();

  /*
  //flush everything and get ready for next loop
   Serial2.flush();
   Serial3.flush();
   Serial.flush();
   */
}


/***********************************************************************************
  ready(): TODO: Why do we need this?
 ***********************************************************************************/
void ready()
{
  HEAD_SERIAL.write('r');
  HEAD_SERIAL.write(myModuleNumber);
}

/************************************************************************************
  propergate(): Properly moves angles downstream and out towards the following module
 ***********************************************************************************/
void propergate()
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

  //*
  Serial.write('\n');
  for(int i=0;i<5;i++)
  {
    Serial.print(horzAngleArray[i]);
  }
  Serial.println(' horz');

  Serial.write('\n');
  for(int i=0;i<5;i++)
  {
    Serial.print(vertAngleArray[i]);
  }
  Serial.println(' vert');
  //*/
}

/************************************************************************************
  move(): Modulates the values to achieve position set points (Runs the PID)
 ***********************************************************************************/

const int MAX_WAIT_TIME = 3000;  // Time to try moving actuator before giving up
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

    PIDcontroller[i].setSetPoint(map(goal,0,255,lowRange[i],highRange[i]));

    int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

    if ((abs(currentAngle-goal)>DEAD_ZONE) &&
       (millis()-horzTimerArray[i]) < MAX_WAIT_TIME)
    {
      analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
      PIDcontroller[i].updateOutput();
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

/****************************************************************************************
  calibrate(): Records sensor values at the hard limits of each vertebrae. Saves to EEPROM.
 *************************************************************************************/
void calibrate()
{

  // HIGH: Move to side when the solenoid selection signal is HIGH
  analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(3000);
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
  analogWrite(MOTOR_CONTROL,MOTOR_SPEED);
  for(int i=0;i<5;i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], LOW);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  delay(3000);
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
      PIDcontroller[i].setEven(true);
      horzAngleArray[i] = '1';
    }
    else
    {
      int temp = highRange[i];
      highRange[i] = lowRange[i];
      lowRange[i] = temp;
      PIDcontroller[i].setEven(false);
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
    EEPROM.write(i*5+6,PIDcontroller[i].getEven());
  }

  straighten();
   
  // Print out calibration
  Serial.println("");
  for(int i=0;i<5;i++){
    Serial.print(highRange[i]);
    Serial.print(' ');
  }
  Serial.println("HIGH ");
  for(int i=0;i<5;i++){
    Serial.print(lowRange[i]);
    Serial.print(' ');
  }
  Serial.println("RANGE ");
  for(int i=0;i<5;i++){
    Serial.print(highRange[i] - lowRange[i]);
    Serial.print(" ");
  }
  Serial.println("LOW ");
  for(int i=0;i<5;i++){
    Serial.print(PIDcontroller[i].getEven(),BIN);
    Serial.print("    ");
  }
  Serial.println("EVEN");
}


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
    PIDcontroller[i].setSetPoint(map(goal,0,255,lowRange[i],highRange[i]));
  }

  // Time limit so if stuck, goes to next
  unsigned long startTime = millis();
  while(inPosition == false && (millis()-startTime) < MAX_WAIT_TIME){
    inPosition = true;

    for(int i=0;i<5;i++){
      int currentAngle = map(analogRead(HORZ_POS_SENSOR[i]),lowRange[i],highRange[i],0,255);

      if(abs(currentAngle-goal)>DEAD_ZONE){
        analogWrite(MOTOR_CONTROL, MOTOR_SPEED);
        PIDcontroller[i].updateOutput();
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
  
  Serial.print("\nManual Control mode entered\n");
  Serial.print("commands: 1-5 to select vertebrae\n");
  Serial.print("          k/l - horizontal actuation\n");
  Serial.print("          i/o - vertical actuation\n");
  Serial.print("          d'v' - adjust actuation delay, where v=s(small),m(medium),l(large)\n");
  Serial.print("          s - stop motor\n");
  Serial.print("          q - quit\n");
  
  while(manual == true)
  {
    if(Serial.available() > 0){
      byteIn = Serial.read();
      
      switch(byteIn)
      {
        case 'c':
          Serial.print("Running calibration...\n");
          calibrate();
          break;
        case '1':
          segSelect = 0;
          StopMov();
          Serial.print("Seg1\n");
          motor = false;
          break;
        case '2':
          segSelect = 1;
          StopMov();
          Serial.print("Seg2\n");
          motor = false;
          break;
        case '3':
          segSelect = 2;
          StopMov();
          Serial.print("Seg3\n");
          motor = false;
          break;
        case '4':
          segSelect = 3;
          StopMov();
          Serial.print("Seg4\n");
          motor = false;
          break;
        case '5':
          segSelect = 4;
          StopMov();
          Serial.print("Seg5\n");
          motor = false;
          break;
        
        case 'd':
          StopMov();
          delaySetting = Serial.read();
          switch (delaySetting)
          {
            case 's':
              actuationDelay = 150;
              Serial.print("Actuation delay set to smallest time (150ms)\n");
              break;
            case 'm':
              actuationDelay = 200;
              Serial.print("Actuation delay set to medium time (200ms)\n");
              break;
            case 'l':
              actuationDelay = 300;
              Serial.print("Actuation delay set to largest time (300ms)\n");
              break;
            default:
              actuationDelay = 200;
              Serial.print("Invalid entry, actuation delay set to medium time (200ms)\n");
              break;
          }
          break;
          
        case 'l':
          if(motor == false)
          {
            StopMov();
            analogWrite(5, MOTOR_SPEED);
            analogWrite(HORZ_ACTUATOR[segSelect], 255);
            digitalWrite(31-segSelect, HIGH);
            Serial.print("l dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break;
         
        case 'k':
          if(motor == false)
          {
            StopMov();
            analogWrite(5, MOTOR_SPEED);
            analogWrite(HORZ_ACTUATOR[segSelect], 255);
            digitalWrite(31-segSelect,LOW);
            Serial.print("k dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break;
        
        case 'i':
          if(motor == false)
          {
            StopMov();
            analogWrite(5, MOTOR_SPEED);
            analogWrite(VERT_ACTUATOR[segSelect], 255);
            digitalWrite(26-segSelect, LOW);
            Serial.print("i dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break; 

        case 'o':
          if(motor == false)
          {
            analogWrite(5, MOTOR_SPEED);
            analogWrite(VERT_ACTUATOR[segSelect], 255);
            digitalWrite(26-segSelect, HIGH);
            Serial.print("o dir\n");
            delay(actuationDelay);
            StopMov();
          }
          break;
        
        case 's':
          StopMov();
          Serial.print("STOPPED\n");
          break;
        
        case 'q':
          manual = false;
          break;
      }//end switch
    }//end if serial
    
    byteIn = 'z';
  }
  Serial.print("\nManual Control mode exited");
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






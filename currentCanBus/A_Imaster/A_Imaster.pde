#include <SPI.h>
#include "interface.h"
#include "boashield_pins.h"
#include "MCP2515.h"

Interface interface(SS);

#define MAX_SLAVES 5
byte lastSlaveID = 0;

#define MASTER_STATE_STOP 0
#define MASTER_STATE_GOING 1
#define MASTER_STATE_STANDBY 2
#define MASTER_STATE_DYING 3

#define MAX_CAN_DELAY 150

byte master_state = MASTER_STATE_STOP;

byte wait_for_slave_init() {
  /*
    wait_for_slave_init() 
      waits two seconds for slaves to come online
      arguments: none
      returns: last id
  */
  // Listen for slave broadcasts
  unsigned long slaveTimer = millis();
  byte initialized[MAX_SLAVES] = {0};
  Serial.println("Wait for slaves");
  Serial.println(slaveTimer);
  Frame slave_message;
  while(millis() - slaveTimer < 5000) {
    slave_message = interface.getMessage();
    if(slave_message.id == 0x00) {
      continue;
    }
    Serial.println(slave_message.data[0],HEX);
    Serial.print(millis());
    Serial.print(" Recieved Slave broadcast from id ");
    Serial.println(slave_message.id,HEX);
    if(slave_message.id < MAX_SLAVES) {
      if(initialized[slave_message.id]) {
        Serial.println("Already initialized, There is an error in slaves");
        break;
      }
      initialized[slave_message.id-1] = 1;
    }
    if(slave_message.id == MAX_SLAVES) {
      break;
    }
    delay(5);
  }
  for(int i = MAX_SLAVES-1; i >= 0; i--) {
    if(initialized[i]) {
      Serial.print("Last slave id ");
      Serial.println(i+1);
      return i+1;
    }
  }
  Serial.println("No slaves appeared to be initialized");
  return 0;
}

Frame block_until_slave_confirm(byte slave, int timeout) {
  Frame slave_message;
  unsigned long slaveTimer = millis();
  while(millis() - slaveTimer < timeout) {
    slave_message = interface.getMessage();
    if(slave_message.id == slave) {
      return slave_message;
    }
  }
  return slave_message;
}

void suicide() {
  Serial.print(millis());
  Serial.println(" Entered Suicide");
  /*master_state = MASTER_STATE_STOP;
  for(int i = 1; i <= lastSlaveID; i++) {
    interface.sendCommand(i,CMD_STOP,PAD,PAD,PAD,PAD,PAD,PAD);
    block_until_slave_confirm(i, MAX_CAN_DELAY);
  }*/
}

void callibrate_all_slaves() {
  Serial.print(millis());
  Serial.println(" Entered Callibrate All");
  for(byte _id = 1; _id <= lastSlaveID; _id++) {
    interface.sendCommand(_id, CMD_PID_CAL, PAD,PAD,PAD,PAD,PAD,PAD);
    //Frame confirm = block_until_slave_confirm(_id,5000);
    //if(confirm.id == 0x00) {
    //  suicide();
    //  break;
    //}
    Serial.print(_id,HEX);
    Serial.println(" Was sent callibration");
  }
  Serial.print(millis());
  Serial.println(" Exit Callibrate All");
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  Serial.println("Entered Setup");
  
  interface.init(MASTER);
  delay(10);
  lastSlaveID = wait_for_slave_init();
  Serial.println("Finished Setup");
  
  //delay(1000);
  //callibrate_all_slaves();
  
  // clear all Serial3 data
  while(Serial3.available()) {
    Serial3.read();
  }
  // start the joystick
  Serial.println("Start joystick");
  Serial3.write(CMD_START);
  
  Serial.println("Enter LOOP");
}

//
// Control data
//
long lastUpdate = 0;
int setPointList[MAX_SLAVES*5] = {0};
int new_angle = -1;
unsigned int dt = 0;
boolean sine_wave = false;
float sine_point = 0.0;


void loop() {
//check for CAN Bus message. this comes first mainly because of reinit
Frame reinit = interface.getMessage();
if(reinit.id || reinit.data[0]) {
  Serial.print(millis());
  Serial.print(": Message from ");
  Serial.println(reinit.id,HEX);
  Serial.println(reinit.data[0],HEX);
}
if(reinit.data[0] == STATE_INITING) {
  for(byte _id = 0; _id < lastSlaveID; _id++) {
    interface.sendCommand(_id, CMD_REINIT, PAD,PAD,PAD,PAD,PAD,PAD);
  }
  Serial.print(millis());
  Serial.println(" Reinitializing from slave");
  interface.init(MASTER);
  lastSlaveID = wait_for_slave_init();
}
  
//check for xbee command. Done second because the buffer isn't going anywhere
//CAN buffer can be overwritten
//also take into consideration state. probably in giant switch itself
Command cmd = CMD_NONE;
if(Serial3.available() >= 3) {
  cmd = (Command)Serial3.read();
  new_angle = Serial3.read();
  dt = Serial3.read();
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(cmd,HEX);
  Serial.print(" ");
  Serial.print(new_angle);
  Serial.print(" ");
  Serial.print(dt);
  //parse xbee command. pro
}

if(Serial.available() ) {
  switch(Serial.read()) {
    case 'i':
      interface.sendCommand(0x01, CMD_REINIT, PAD,PAD,PAD,PAD,PAD,PAD);
      Serial.print(millis());
      Serial.println(" Reinitializing Manual");
      interface.init(MASTER);
      lastSlaveID = wait_for_slave_init();
      break;
    case 'c':
      callibrate_all_slaves();
      break;
    case 's':
      new_angle = 45;
      dt = 1000;
      cmd = CMD_START;
      break;
    case 'm':
      new_angle = 5;
      dt = 1000;
      cmd = CMD_START;
      break;
    case 'M':
      new_angle = 0;
      dt = 300;
      cmd = CMD_START;
      sine_wave = true;
      break;
    case 'S':
      cmd = CMD_STOP;
      sine_wave = false;
      sine_point = 0.0;
      break;
  }
}

switch(cmd) {
  case CMD_START:
    Serial.println("Joystick sent start");
    for(int i = 1; i <= lastSlaveID; i++) {
      // Start in closed loop mode
      interface.sendCommand(i,CMD_START,0x22,PAD,PAD,PAD,PAD,PAD);
      Frame confirm = block_until_slave_confirm(i,MAX_CAN_DELAY);
      interface.sendCommand(i,CMD_SET_PID,5,5,0,0,PAD,PAD);
      block_until_slave_confirm(i, MAX_CAN_DELAY);
      if(confirm.id == 0x00) {
        master_state = MASTER_STATE_STOP;
        Serial.print(millis());
        Serial.print(" Suicide data(from state stop) ");
        Serial.println(confirm.data[0],HEX);
        suicide();
      }
    }
    // if everything was successfull set the new state
    master_state = MASTER_STATE_GOING;
    break;
  case CMD_STOP:
    Serial.println("Joystick sent stop");
    master_state = MASTER_STATE_STOP;
    for(int i = 1; i <= lastSlaveID; i++) {
      interface.sendCommand(i,CMD_STOP,PAD,PAD,PAD,PAD,PAD,PAD);
      block_until_slave_confirm(i, MAX_CAN_DELAY);
    }
    break;
  case CMD_SET_ANGLEX:
    break;
  case CMD_SET_ANGLEZ:
    break;
  case CMD_NONE:
    break;
  case CMD_PID_CAL:
    callibrate_all_slaves();
    break;
  // May get out of sync with the joystick
  // Clear all Serial3 data
  // Should be back in sync
  default:
    while(Serial3.available()) {
      Serial3.read();
    }
    break;
}

switch(master_state) {
  case MASTER_STATE_GOING:
    if(!dt) {
      break;
    }
    if(millis() - lastUpdate > dt) {
      for(int i = MAX_SLAVES*5-1; i > 0; i--) {
        setPointList[i] = setPointList[i-1];
      }
      setPointList[0] = new_angle;
      lastUpdate = millis();
      for(int i = 1; i <= lastSlaveID; i++) {
        Serial.print(millis());
        Serial.print(" Send command to ");
        Serial.println(i);
        Serial.print(setPointList[(i-1)*5]);Serial.print(" ");
        Serial.print(setPointList[(i-1)*5+1]);Serial.print(" ");
        Serial.print(setPointList[(i-1)*5+2]);Serial.print(" ");
        Serial.print(setPointList[(i-1)*5+3]);Serial.print(" ");
        Serial.print(setPointList[(i-1)*5+4]);Serial.print(" ");
        Serial.println();
        interface.sendCommand(i,CMD_SET_ANGLEX,setPointList[(i-1)*5],setPointList[(i-1)*5+1],setPointList[(i-1)*5+2],setPointList[(i-1)*5+3],setPointList[(i-1)*5+4],PAD);
        Frame confirm = block_until_slave_confirm(i,MAX_CAN_DELAY);
        interface.sendCommand(i,CMD_GET_SETPOINT,PAD,PAD,PAD,PAD,PAD,PAD);
        Frame confirm2 = block_until_slave_confirm(i,MAX_CAN_DELAY);
        interface.sendCommand(i,CMD_GET_OUTPUT,PAD,PAD,PAD,PAD,PAD,PAD);
        Frame confirm3 = block_until_slave_confirm(i,MAX_CAN_DELAY);
        
        if(sine_wave) {
          // New point
          new_angle = 20.0*sin(sine_point)+20;
          sine_point += 0.5;
        }
        
        // confirmation fram will have id 0x00 if the slave did not respond by MAX_CAN_DLELAY
        if(confirm.id == 0x00) {
          master_state = MASTER_STATE_STOP;
          Serial.print(millis());
          Serial.print(" Suicide data ");
          Serial.println(confirm.data[0],HEX);
          suicide();
          break;
        }
        else {
          // Sensor Data
          Serial.print(millis());Serial.print(" ");
          Serial.print(confirm.id,HEX);Serial.print(" ");
          Serial.print(confirm.data[0],HEX);Serial.print(" ");
          Serial.print(confirm.data[1],DEC);Serial.print(" ");
          Serial.print(confirm.data[2],DEC);Serial.print(" ");
          Serial.print(confirm.data[3],DEC);Serial.print(" ");
          Serial.print(confirm.data[4],DEC);Serial.print(" ");
          Serial.println(confirm.data[5],DEC);
          
          // Set point data
          Serial.print(millis());Serial.print(" ");
          Serial.print(confirm2.id,HEX);Serial.print(" ");
          Serial.print(confirm2.data[0],HEX);Serial.print(" ");
          Serial.print(confirm2.data[1],DEC);Serial.print(" ");
          Serial.print(confirm2.data[2],DEC);Serial.print(" ");
          Serial.print(confirm2.data[3],DEC);Serial.print(" ");
          Serial.print(confirm2.data[4],DEC);Serial.print(" ");
          Serial.println(confirm2.data[5],DEC);
          
          // Output data
          Serial.print(millis());Serial.print(" ");
          Serial.print(confirm3.id,HEX);Serial.print(" ");
          Serial.print(confirm3.data[0],HEX);Serial.print(" ");
          Serial.print(confirm3.data[1],HEX);Serial.print(" ");
          Serial.print(confirm3.data[2],HEX);Serial.print(" ");
          Serial.print(confirm3.data[3],HEX);Serial.print(" ");
          Serial.print(confirm3.data[4],HEX);Serial.print(" ");
          Serial.println(confirm3.data[5],HEX);
        }
      }
    }
    break;
  case MASTER_STATE_STANDBY:
    break;
  case MASTER_STATE_STOP:
    break;
  case MASTER_STATE_DYING:
    break;
}

//have to include states somewhere
/*###################   state_going   ###################
everything is fair game here. only restriction is that calibrate is true
calibrate true tree works cause anying updating is overwritten
maybe i should throw in a motor trigger condition just to be safe
#########################################################*/

/********************james says stop and standby are essentially the same thing
###################   state_standby   ###################
motor is off
setpoints are being constantly set as current
everything still going and ready to go
#########################################################
*******************************************************************************
*/

/*###################   state_stop    ###################
basically arduino only state
lights will probably be on still
#########################################################*/

/*###################   state_dying   ###################
everything is turning off
last action before arduino dies is sending state?
#########################################################*/

  // Check battery state


}

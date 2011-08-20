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

#define MAX_CAN_DELAY 25

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
  Frame slave_message;
  while(millis() - slaveTimer < 2000) {
    slave_message = interface.getMessage();
    if(slave_message.id == 0x00) {
      continue;
    }
    Serial.print("Recieved Slave broadcast from id ");
    Serial.println(slave_message.id,HEX);
    if(slave_message.id == MAX_SLAVES) {
      break;
    }
  }
  return slave_message.id;
}

Frame block_until_slave_confirm(int timeout) {
  Frame slave_message;
  unsigned long slaveTimer = millis();
  while(millis() - slaveTimer < timeout) {
    slave_message = interface.getMessage();
    if(slave_message.id != 0x00) {
      return slave_message;
    }
  }
  return slave_message;
}

void suicide() {
  for(int i = 0; i < lastSlaveID; i++) {
    interface.sendCommand(i+1,CMD_STOP,PAD,PAD,PAD,PAD,PAD,PAD);
    block_until_slave_confirm(MAX_CAN_DELAY);
  }
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

//what exactly will the controller send to the master to make it trigger the initialization command
//I don't want this to initialize until there is a controller present

  // Wait for initialization command from controller
  while (Serial3.available() < 2) {
    delay(10);
  }
  
  Serial.read();
  
  interface.init(MASTER);
  delay(10);
  lastSlaveID = wait_for_slave_init();
  delay(100);
  Serial.println("Finished Setup");
}

//
// Control data
//
long lastUpdate = 0;
int setPointList[MAX_SLAVES*5] = {0};
int new_angle = -1;
unsigned int dt = 0;


void loop() {

//check for CAN Bus message. this comes first mainly because of reinit
Frame reinit = interface.getMessage();
if(reinit.data[0] == CMD_REINIT) {
  for(byte _id = 0; _id < lastSlaveID; _id++) {
    interface.sendCommand(_id, CMD_REINIT, PAD,PAD,PAD,PAD,PAD,PAD);
  }
  interface.init(MASTER);
  wait_for_slave_init();
}
  
//check for xbee command. Done second because the buffer isn't going anywhere
//CAN buffer can be overwritten
//also take into consideration state. probably in giant switch itself
Command cmd = CMD_NONE;
if(Serial3.available() >= 4) {
  cmd = (Command)Serial3.read();
  new_angle = Serial3.read();
  dt = Serial3.read();
  Serial3.read();
  
  //parse xbee command. pro
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
      for(int i = 0; i < lastSlaveID; i++) {
        interface.sendCommand(i+1,CMD_SET_ANGLEX,setPointList[i*5],setPointList[i*5+1],setPointList[i*5+2],setPointList[i*5+3],setPointList[i*5+4],PAD);
        Frame confirm = block_until_slave_confirm(MAX_CAN_DELAY);
        // confirmation fram will have id 0x00 if the slave did not respond by MAX_CAN_DLELAY
        if(confirm.id == 0x00) {
          master_state = MASTER_STATE_STOP;
          suicide();
          break;
        }
      }
    }
    break;
  case MASTER_STATE_STANDBY:
    break;
  case MASTER_STATE_STOP:
    if(cmd == CMD_START) {
      for(int i = 0; i < lastSlaveID; i++) {
        interface.sendCommand(i+1,CMD_START,PAD,PAD,PAD,PAD,PAD,PAD);
        Frame confirm = block_until_slave_confirm(MAX_CAN_DELAY);
        if(confirm.id == 0x00) {
          master_state = MASTER_STATE_STOP;
          suicide();
        }
      }
      // if everything was successfull set the new state
      master_state = MASTER_STATE_GOING;
    }
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
}

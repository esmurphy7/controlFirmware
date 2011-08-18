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
  
  // Wait for initialization command from controller
  while (Serial3.available() < 2) {
    delay(10);
  }
  
  Serial.read();
  
  interface.init(MASTER);
  delay(10);  
  // Listen for slave broadcasts
  Frame slave_message;
  while(init_enable) {
    slave_message = interface.getMessage();
    if(slave_message.id == 0x00) {
      continue;
    }
    Serial.print("Recieved Slave broadcast from id ");
    Serial.println(slave_message.id,HEX);
    if(slave_message.id == num_slaves) {
      break;
    }
  }
  delay(100);
  Serial.println("Finished Setup");
}

void loop(){

//check for CAN Bus message. this comes first mainly because of reinit

  
//check for xbee command. Done second because the buffer isn't going anywhere
//CAN buffer can be overwridden
//also take into consideration state. probably in giant switch itself

//parse xbee command. pro

//have to include states somewhere
/*###################   state_going   ###################
everything is fair game here. only restriction is that calibrate is true
calibrate true tree works cause anying updating is overwritten
maybe i should throw in a motor trigger condition just to be safe
#########################################################*/

/*###################   state_standby   ###################
motor is off
setpoints are being constantly set as current
everything still going and ready to go
#########################################################*/

/*###################   state_stop    ###################
basically arduino only state
lights will probably be on still
#########################################################*/

/*###################   state_dying   ###################
everything is turning off
last action before arduino dies is sending state?
#########################################################*/
}

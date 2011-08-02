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
  
  init_enable = Serial3.read();
  num_slaves = Serial3.read();
  
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

//check for xbee command
//also take into consideration state. probably in giant switch itself

//parse xbee command. pro



}
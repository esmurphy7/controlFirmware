#include "interface.h"
#include "MCP2515_defs.h"
#include "MCP2515.h"

Interface::Interface(int cs): CAN(cs, 0) {
  _id = 0x00;
  arg[0] = 0x00;
  arg[1] = 0x00;
  arg[2] = 0x00;
  arg[3] = 0x00;
  arg[4] = 0x00;
  arg[5] = 0x00;
}

void Interface::init() {
  // Initialise MCP2515 CAN controller at the specified speed and clock frequency
  // In this case 1Mbps with a 16MHz oscillator
  // (Note:  This is the oscillator attached to the MCP2515, not the Arduino oscillaltor)
  int baudRate=CAN.Init(1000,16);
  Serial.println(CAN.Mode(MODE_NORMAL),HEX);
  if(baudRate>0) {
    Serial.println("MCP2515 Init OK ...");
    Serial.print("Baud Rate (kbps): ");
    Serial.println(baudRate,DEC);
  } else {
    Serial.println("MCP2515 Init Failed ...");
  }
  Serial.println(CAN.Read(CANSTAT),HEX);
  Serial.println("Ready ...");
  
  //arduino id assignment
  
  while(Serial2.available()){
    Serial2.read();
  }
  
  while(! _id){  
    
    if(Serial2.available()){
      
      _id = Serial2.read() + 0x01;
      Serial.print("recieved id ");
      Serial.println(_id,HEX);
      byte _state = Serial2.read();
      Serial.println(_state,HEX);
      
      if(_state != STATE_ALIVE){
        _id=0x00;
        Serial.println("ARGH");
        continue;
      }
      delay(50);
      sendDown();
      sendState(STATE_ALIVE,PAD,PAD,PAD,PAD,PAD,PAD);
    }
  }
  Serial.println("i has finished init");
}

char Interface::getID() {
  return _id;
}

char Interface::getArg(int id) {
  return arg[id];
}
void Interface::sendCommand(char id, Command cmd, char arg1, char arg2, char arg3, 
                                                  char arg4, char arg5, char res) {
  Frame message;
  message.id = id;
  message.srr = 0x00;
  message.rtr = 0x00;
  message.ide = 0x00;
  message.dlc = 0x08;
  message.data[0] = cmd;
  message.data[1] = arg1;
  message.data[2] = arg2;
  message.data[3] = arg3;
  message.data[4] = arg4;
  message.data[5] = arg5;
  message.data[6] = res;
  message.data[7] = 0x00;
  CAN.LoadBuffer(TXB0, message);
  CAN.SendBuffer(TXB0);
}
void Interface::sendState(State state, char arg1, char arg2, char arg3, 
                                       char arg4, char arg5, char res) {
  Frame message;
  message.id = _id; // Send our own ID in the id slot (the master will listen for all ids
  message.srr = 0x00;
  message.rtr = 0x00;
  message.ide = 0x00;
  message.dlc = 0x08;
  message.data[0] = state;
  message.data[1] = arg1;
  message.data[2] = arg2;
  message.data[3] = arg3;
  message.data[4] = arg4;
  message.data[5] = arg5;
  message.data[6] = res;
  message.data[7] = 0x00;
  CAN.LoadBuffer(TXB0, message);
  CAN.SendBuffer(TXB0);
}

void Interface::sendUp() {
  Serial2.write(_id);
  Serial2.write(STATE_ALIVE);
}

void Interface::sendDown() {
  Serial1.write(_id);
  Serial1.write(STATE_ALIVE);
}

void Interface::broadcast() {
  sendUp();
  sendDown();
  sendState(STATE_ALIVE, PAD, PAD, PAD, PAD, PAD, PAD);
}

Frame Interface::getMessage() {
  Frame message;
  message.id = 0x00;
  message.data[0] = 0x00;
  byte int_byte = CAN.Read(CANINTF);
  if(int_byte) {
//    Serial.println("Interupted");
    // determine which interrupt flags have been set
    byte interruptFlags = CAN.Read(CANINTF);

    if(interruptFlags & RX0IF) {
	// read from RX buffer 0
	message = CAN.ReadBuffer(RXB0);
    }
    if(interruptFlags & RX1IF) {
	// read from RX buffer 1
	message = CAN.ReadBuffer(RXB1);
	// (this is poor code as clearly if two messages are received then the second will overwrite the first)
    }
    if(interruptFlags & TX0IF) {
	// TX buffer 0 sent
    }
    if(interruptFlags & TX1IF) {
	// TX buffer 1 sent
    }
    if(interruptFlags & TX2IF) {
	// TX buffer 2 sent
    }
    if(interruptFlags & ERRIF) {
	// error handling code
    }
    if(interruptFlags & MERRF) {
	// error handling code
	// if TXBnCTRL.TXERR set then transmission error
	// if message is lost TXBnCTRL.MLOA will be set
    }
 /*   Serial.print(message.id,HEX);
    Serial.print(" ");
    Serial.print(_id,HEX);
    Serial.println();*/
  }
  // Cast message.id to byte here since it isn't actually returned as a byte
  if((byte)message.id == _id) {
    return message;
  }
  else {
    Frame none;
    none.id = 0x00;
    none.data[0] = CMD_NONE;
    return none;
  }
}

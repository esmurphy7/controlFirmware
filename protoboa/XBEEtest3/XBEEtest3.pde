/*************
 * this is a thing, it does stuff
 * Program receives packets, does stuff to them, forwards them if necesssary and responds to cmds
 * Current commands accepted by system (cmd  = CPU->arduino, response= arduino->CPU):
 * ! = packet start
 * = packet end
 * #(ID number) = all following bytes are intented for (ID number) only (cmd), the following is from (ID number) (response)
 * S = requesting sensor data (cmd), the following is sensor data (response) data will always be sent in pin order 0-15 (analog) and 20-53 (digital)
 * A(acutor possition data) = following are actuator possition cmds (note actuator data always transmitted in pin order 2-13 (pwm)
 * E(code) = error of type (code) has occured (can be either a cmd or a response).
 * 
 * example cmd: !#0001SA1203A1209A1313A1213* <-note that A always follows an ID
 * this requests current sensor data from all arduinos and sends actuator cmds for arduinos #1
 * 
 * note: !#5-S* only requests arduino 5's sensor data
 * 
 * example response: !#0001S1231S1531S123S0445E0001*
 * this sends sensor data for arduino #1 and informs that arduino #1 has experienced and error of type 1.
 * 
 * NOTE - numbers will be sent as int, hence on the serial monitor they will probably look like garble, but as long 
 * as you store them as int on the other end it should be fine. 
 * NOTE AGAIN - IDs,sensors/actuators need to be 4 digits long, pad them if they arn't. Errors are 2 digit long
 *****************/

#define CMD_SIZE 10

unsigned char inByte;
unsigned char outByte;
unsigned char inArray[CMD_SIZE];
unsigned char fwdArray[CMD_SIZE];
unsigned char outArray[CMD_SIZE];

int pwmArray[11]={
  0};
int analogArray[15]={
  0};
int digitalArray[20]={
  0}; //not sure how many we are using

int IDnumber=1;
int number;

int i;
int k;

boolean com_packetError=false;
boolean com_packetReceived=false;

boolean cmd_sensorData=false;

int analogValue;

int ERROR=0;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  /*things to add here are:
   the arduinos need to initialize where they are in the snake, 
   also initiallize their slaves etc. In particular they need to have
   a unique ID*/
}

void loop() {
  /*************************RECIEVE-Receives packets from central CPU*****************************/

  if(Serial3.available()){ //receive data
    inByte=Serial3.read();
    if(inByte=='!'){
      Serial3.println("PACKET INCOMMING");
      i=0;
      while(com_packetReceived==false){
        if(i>CMD_SIZE){
          Serial3.println("ERROR PACKET TOO BIG"); //if packet exceeds expected length send error, this includes the end byte but not the first byte
          delay(10); 
          Serial3.flush();
          com_packetError=true;
          break;
        }
        inByte=Serial3.read();
        inArray[i] = inByte; //save the packet to an array
        i++;

        if(inByte=='*'){ //if we get to the end of the package, then marked as received and stop reading
          com_packetReceived==true;
          Serial3.println("PACKET RECIEVED");
          break;
        }

        if(inByte =='!'){
          Serial3.println("ERROR NO END BYTE"); //if packet has no end byte send error
          delay(10);
          Serial3.flush();
          com_packetError=true;
          break;
        }
      }
    }
  }
  if(com_packetReceived==true){//this probably won't be in the final program, sends the received packet back for error checking purposes
    Serial3.print("!");
    for(i=0;i<CMD_SIZE;i++){
      outByte=inArray[i];
      Serial3.print(outByte);
      delay(10);
    }
  }
  /************************INTERPRET - Interprets the packet received*******************/

  if(com_packetReceived==true){
    i=0; //indexes inArray
    k=0; //indexes pwmArray
    Serial3.print("INTERPRET PACKET");
    while(inArray[i]!='*'){
      if(inArray[i]=='#'){ //read the next 2 chars to determine ID#
        number=number_read();
        i++;
      }

      if(number==IDnumber){ //if the IDs match pay attention to signal

        switch (inArray[i]){ //this defines how it will respond to char cmds, add new ones here as needed
        case 'S':
          cmd_sensorData=true;
          break;

        case 'A':
          pwmArray[k]=number_read();
          k++;
          break;

        case 'E':
          ERROR = number_read();
          //put error response here when we know what that response should be
          break;

        default:
          break;

        }
      } 

      i++;
    }
    com_packetReceived==false;
  }
  /**********************************SEND and (eventually)FORWARD*******************************/
  //currently just sends analog data since we don't know which digital pins we're using
  if(cmd_sensorData==true){
    Serial3.print('!');
    delay(10);
    Serial3.print('#');
    delay(10);
    Serial3.print(0);
    delay(10);
    if(IDnumber<100){
      Serial3.print(0);
    }
    if(IDnumber<10){
      Serial3.print(0); 
    }
    Serial3.print(IDnumber);
    i=0;
    while(i<16){
      analogValue=analogRead(i);
      Serial3.print('S');
      delay(10);
      Serial3.print(analogValue);
      delay(10);
      i++;
    }
    Serial3.print('*');
    cmd_sensorData=false;
  }
}

int number_read(){ 
  //this reads the next 4 values in inArray and turns them into an int
  //increments i to the last valeu of number
  i++;
  number = (inArray[i]-48)*1000+(inArray[i+1]-48)*10+(inArray[i+2]-48)*10+(inArray[i+3]-48);
  i=i+3;
  return number;
}











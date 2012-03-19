/*A_5SegmentDigitalTiming controllor
used at vancouver 125 celebrations
various messy improvs

Created July 10, 2011 Julian Fong
*/
import processing.serial.*;
import controlP5.*;
//requires adding of controlP5 to libary folder
//or for temperary fix, under "Sketch" on menu bar "Add File..." 

// serial port
Serial myPort;

boolean ready = true;

Toggle autoToggle;

ControlP5 controlP5;

RadioButton directionButton;

float prevSentTime = 0;

int leftNumber = 5;
int rightNumber = 5;
int turning = 1;
int direction = 0;

Textlabel leftText, rightText, turningText;

void setup() {
  size(400, 400);
  smooth();
  
  controlP5 = new ControlP5(this);
  
  // List all the available serial ports
  println(Serial.list());
  
  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[0], 115200);
  
  frameRate(30);
  
  directionButton = controlP5.addRadioButton("radioButton",50,100);
  directionButton.setItemsPerRow(2);
  directionButton.setSpacingColumn(200);
  directionButton.setSize(50,50);
  directionButton.addItem("LEFT",0);
  directionButton.addItem("RIGHT",1);
  directionButton.deactivateAll();
  
  autoToggle = controlP5.addToggle("AUTO", false, 175, 200, 50, 50);
}

void draw(){
  background(190);
  
  text("AUTO settings(commented out for public use, only arrow keys)\nq, a for left\ne, d for right\nw, s for phase\nz, c for direction",10,340);
  text(leftNumber,75,170);
  text(rightNumber,325,170);
  text(turning,180,195);
  if(direction == 0){
    text("left",180,180);
  }
  else{
    text("right",180,180);
  }
  
  if(directionButton.value() != -1 && 
      (ready == true || (millis()-prevSentTime)>3000) ||
      ((millis()-prevSentTime)>3000 && autoToggle.value()==1) ){
    ready = false;
    if(round(directionButton.value()) == 0){
      myPort.write('1');
      println("sent '1'");
    }
    else{
      myPort.write('0');
      println("sent '0'");
    }
    
    myPort.clear();
    directionButton.deactivateAll();
    prevSentTime = millis();
    
    //AUTO******************
    if(autoToggle.value()==1){
      turning++;
      if(direction == 0){
        if(turning > leftNumber){
          turning = 1;
          direction = 1;
        }
      }
      else{
        if(turning > rightNumber){
          turning = 1;
          direction = 0;
        }
      }
      directionButton.activate(direction);
    }
  }
}

void keyPressed() {
  /*
  if (keyCode == LEFT) {
    directionButton.activate(0);
  }
  else if (keyCode == RIGHT) {
    directionButton.activate(1);
  }
  
  if(key == 'q'){
    leftNumber++;
  }
  else if(key == 'a'){
    leftNumber--;
  }
  leftNumber = constrain(leftNumber,1,10);
  
  if(key == 'e'){
    rightNumber++;
  }
  else if(key == 'd'){
    rightNumber--;
  }
  rightNumber = constrain(rightNumber,1,10);
  
  if(key == 'w'){
    turning++;
  }
  else if(key == 's'){
    turning--;
  }
  turning = constrain(turning,0,10);
  
  if(key == 'z'){
    direction = 0;
  }
  else if(key == 'c'){
    direction = 1;
  }
  //*/
  if (keyCode == DOWN) {
    leftNumber = 5;
    rightNumber = 5;
    autoToggle.setValue(0);
    directionButton.deactivateAll();
  }
  else if (keyCode == UP) {
    leftNumber = 5;
    rightNumber = 5;
    autoToggle.setValue(1);
  }
  else if (keyCode == RIGHT) {
    leftNumber = 3;
    rightNumber = 5;
    autoToggle.setValue(1);
  }
  else if (keyCode == LEFT) {
    leftNumber = 5;
    rightNumber = 3;
    autoToggle.setValue(1);
  }
}

void keyReleased(){
  autoToggle.setValue(0);
  //directionButton.deactivateAll();
}

void serialEvent(Serial myPort){
  char readByte = (char)myPort.read();
  println("receive '"+readByte+"'");
  if(readByte == '1'){
    ready = true;
    println("ready");
  }
  myPort.clear();
}

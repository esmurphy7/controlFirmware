import processing.serial.*;
import processing.opengl.*;

long timer =0;

Serial port;

int[] analogArray = new int[17];
int[] pwmArray = new int[12];
int[] digitalArray = new int[10];

String[] cpwmArray = new String[12];
String[] cdigitalArray = new String[10];

String inString;
String saveString;
String outString;
String tempString;
String statusString ="null";

char[] tempChars;

PrintWriter output;

Phrase courier14;
Phrase courier14B;


int msgLength;
int ID = 0001;

int i; //general incrment
int k =16; //analog array increment
int j = 12; //pwm array increment
int h = 10; //digital array increment

boolean msgReceived = false;
boolean msgParsed = false;
boolean msgSaved = false;
boolean msgGood = true;
boolean msgSent = false;

boolean start = false;

boolean guiActive = false;

boolean[] guiActiveArray;
color[] guiColorArrray;

PGraphics hitBuffer;

Textfield[] textfeildArray;

void setup() {
  size(640,480,JAVA2D);
  //setup hit screen
  hitBuffer = createGraphics(width,height,P2D);
  //setup Serial
  println(Serial.list()); 
  port = new Serial(this, Serial.list()[1], 9600);
  port.bufferUntil('*');

  //setup file I/O
  output = createWriter("serialData.txt");

  //setup phrases
  courier14 = new Phrase("","CourierNewPSMT-14.vlw");
  courier14B = new Phrase("","CourierNewPS-BoldMT-14.vlw",color(0,0,0),14);

  //setup test fields
  guiColorArrray = new color[22];
  guiActiveArray = new boolean[22];
  textfeildArray = new Textfield[22];
  for(i=0;i<22;i++) {
    textfeildArray[i] = new Textfield(4,i);
    guiColorArrray[i] = color(i);
    guiActiveArray[i] = false;
  }
}

void draw() {
  /******************DISPLAY*******************/
  background(255,255,255);
  hitBuffer.background(color(0,1));
  pushMatrix();
  translate(width/2,height/2);

  //Program progress displays
  courier14B.updateString("Status: "+statusString);
  courier14B.display(-310,-130);

  //incomming serial
  courier14B.updateString("Serial: "+inString);
  courier14B.display(-310,-115);

  //Sensor displays
  courier14B.updateString("Pin Status:");
  courier14B.display(-310,-94);

  for(i=0;i<k;i++) {
    courier14.updateString("S"+nf(i,2)+": "+nf(analogArray[i],4));
    courier14.display(-310,-94+18*(i+1));
  }
  for(i=0;i<j;i++) {
    courier14.updateString("PWM"+nf(i,2)+": "+nf(pwmArray[i],4));
    courier14.display(-215,-94+18*(i+1));
  }
  for(i=0;i<(h-4);i++) {
    courier14.updateString("Digital"+nf(i+22,2)+": "+nf(digitalArray[i],4));
    courier14.display(-95,-94+18*(i+1));
  }

  //gui display
  courier14B.updateString("Input Values:");
  courier14B.display(130,-200);

  for(i=0;i<12;i++) {
    courier14.updateString("PWM"+nf(i+2,2)+": ");
    courier14.display(130,-200+18*(i+1));
    textfeildArray[i].display(190,-214+18*(i+1));
  }

  for(i=12;i<22;i++) {
    courier14.updateString("Digital"+nf(i+10,2)+": ");
    courier14.display(130,-182+18*(i+1));
    textfeildArray[i].display(215,-196+18*(i+1));
  }

  //debug displays
  courier14.updateString("mouseX: "+nfs(mouseX-width/2,4) +" "+"mouseY: "+nfs(mouseY-height/2,4)); 
  courier14.display(-310,-220);
  courier14.updateString("Framerate: "+nf(frameRate,2,1)); 
  courier14.display(-310,-202);
  courier14.updateString("hitColor: "+hex(hitBuffer.get(mouseX,mouseY))); 
  courier14.display(-310,-184);

  //start, stop exit button
  fill(76,76,255);
  rect(-255,-173,45,20);
  courier14.updateString("STOP"); 
  courier14.display(-250,-156);

  fill(76,255,76);
  rect(-315,-173,45,20);
  courier14.updateString("START"); 
  courier14.display(-313,-156);

  fill(255,76,76);
  rect(-200,-173,45,20);
  courier14.updateString("EXIT"); 
  courier14.display(-195,-156);

  hitBuffer.beginDraw();
  hitBuffer.noStroke();
  hitBuffer.fill(255,0,0);
  hitBuffer.rect(-200+width/2,-173+height/2,45,20);
  hitBuffer.fill(0,255,0);
  hitBuffer.rect(-310+width/2,-173+height/2,45,20);
  hitBuffer.fill(0,0,255);
  hitBuffer.rect(-255+width/2,-173+height/2,45,20);
  hitBuffer.endDraw();

  popMatrix(); 


  /********************MESSAGE HANDLING******************/
  if(start == true) {
    if(msgReceived == true) {
      statusString = parseString(inString);
    }
    if(msgParsed == true) {
      saveData();
    }
    if(msgSent == false) {
      sendCMD();
    }
  }
}

/*****************serialEvent********************/
void serialEvent(Serial port) {
  inString = port.readString();
  println(inString);
  tempChars = inString.toCharArray();
  msgReceived = true;
  msgSent = false;
}

/*******************PARSE MESSAGE******************************/
String parseString(String string) {
  if(tempChars[0] != '!') {
    string = "Error: No start byte";
    port.clear();
    msgGood = false;
    return string;
  }
  msgLength = inString.length();
  if(tempChars[msgLength-1] != '*') {
    string = "Error: No end byte";
    port.clear();
    msgGood = false;
    return string;
  }
  if(tempChars[1] != '#') {
    string = "Error: No ID";
    port.clear();
    msgGood = false;
    return string;
  }
  tempString = string.substring(2,6);
  ID = stringTOint(tempString);
  k=0;
  j=0;
  for(i=6;i<msgLength;i++) {
    analogArray =zeroArray(analogArray);
    analogArray =zeroArray(analogArray);
    tempString ="";
    switch(tempChars[i]) {
    case 'S':
      tempString.copyValueOf(tempChars,i+1,i+5);
      analogArray[k] = stringTOint(tempString);
      i+=6;
      k++;
      break;

    case 'P':
      tempString.copyValueOf(tempChars,i+1,i+5);
      pwmArray[j] = stringTOint(tempString); 
      i+=6;
      j++;
      break;

    case 'D':
      tempString.copyValueOf(tempChars,i+1,i+5);
      digitalArray[h] = stringTOint(tempString); 
      i+=6;
      h++;
      break;

    default:
      string = "Error: unknown byte";
      return string;
    }
  }

  msgReceived = false;
  msgParsed = true;
  msgGood = true;
  string = "Message received";
  return string;
}

/***************************SAVE DATA******************************/

void saveData() {
  print("Arduino "+nf(ID,3)+ ","+ "Time = "+nf(theTime(),5,3));
  for(i=0;i<k;i++) {
    print(","+"S"+"("+nf(i,2)+")"+nf(analogArray[i],4));
  }
  for(i=0;i<j;i++) {
    print(","+"A"+"("+nf(i,2)+")"+nf(pwmArray[i],4));
  }
  println("");
  statusString = "Message saved";
  msgParsed = false;
  msgSaved = true;
}

/************************sendCMD***************************/

void sendCMD() {
  if(msgGood == true) {
    createCMD();
    tempString = "!#"+nf(ID,4)+"S"+"P"+join(cpwmArray,'P')+"D"+join(cdigitalArray,'D')+"*";
    port.write(tempString);
    println(tempString);
  }
  if(msgGood == false) {
    tempString = "!#"+nf(ID,4)+"E*";
    port.write(tempString);
  }
  msgSent = true;
}

/********************createCMD********************************/
void createCMD() {
  int tempInt;
  for(i=0;i<12;i++) {
    tempInt =textfeildArray[i].getValue();
    cpwmArray[i] = nf(tempInt,4);
  }
  for(i=12;i<22;i++) {
    tempInt =textfeildArray[i].getValue();
    cdigitalArray[i-12] = nf(tempInt,4);
  }
}

/**********************keyPressed*******************/
void keyPressed() {
  if((key >='0' && key <= '9') || key == ENTER) {
    if(guiActive == true) {
      for(i=0;i<22;i++) {
        if(guiActiveArray[i] == true) {
          textfeildArray[i].update();
          break;
        }
      }
    }
  }
}
/**********************mouseClicked***************/
void mousePressed() {
  color hitColor =hitBuffer.get(mouseX,mouseY);
  if(hitColor == color(255,0,0)) {
    port.stop();
    output.flush();
    output.close();
    exit();
  }
  if(hitColor == color(0,255,0)) {
    start = true;
  }
  if(hitColor == color(0,0,255)) {
    start = false;
    msgSent = false;
  }
  if(hitColor == color(0,1)) {
    for(i=0;i<22;i++) {
      guiActiveArray[i]= false;
    }
    guiActive = false;
  }
  else {
    if( guiActive == false) {
      for(i=0;i<22;i++) {
        if( hitColor == guiColorArrray[i]) {
          guiActiveArray[i] = true;
          guiActive = true;
          break;
        }
      }
    }
    else {
      for(i=0;i<22;i++) {
        guiActiveArray[i]= false;
      }
      for(i=0;i<22;i++) {
        if( hitColor == guiColorArrray[i]) {
          guiActiveArray[i] = true;
          break;
        }
      }
    }
  }
}
/************************stringTOint******************************/
int stringTOint(String inString) {
  int number =0;
  int digits = inString.length();
  for(i=0;i<digits;i++) {
    number+=(int(inString.charAt(i))-48)*pow(10,(digits-i-1));
  }
  return number;
}

/*********************theTime***************************/

float theTime() {
  float time;
  timer -= millis();
  time = timer/1000;
  return time;
}

/******************zeroArray********************/
int[] zeroArray(int[] inArray) {
  int arrayLength = inArray.length;
  for(i=0;i<arrayLength;i++) {
    inArray[i] = 0;
  }
  return inArray;
}


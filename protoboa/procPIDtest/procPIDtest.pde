import processing.opengl.*;
import processing.serial.*;

Serial port;

PrintWriter saveFile;

int strokeLength;
int oldStrokeLength;
int strokeSensor;
int savedFiles = 0;
float timerStart;
float time;
int latency =0;
int latencyStart;
int k;
float t=0;

char[] inMSG =new char[6];

PGraphics guiHitBuffer;
boolean guiEngaged = false;
boolean received = false;
boolean start = false;
boolean exitApp = false;
boolean saveData = false;
color hitColor;

PFont font;

Slider slider;

int i;

void setup() {
  size(250,118,OPENGL);

  println(Serial.list());
  port = new Serial(this,Serial.list()[1],115200);
  port.bufferUntil('*');

  guiHitBuffer = createGraphics(width,height,P2D);
  guiHitBuffer.background(0);

  slider = new Slider(125,68,0,int(width*0.8),int(height*0.20),color(255,41,41));
  font = loadFont("CourierNewPSMT-20.vlw");
  textFont(font);

  slider.setValue(0.5);
}

void draw() {
  background(255,255,255); 
  slider.display();

  if(guiEngaged == true ) {
    slider.update();
  }
  /*if(received == false) {
   // latencyStart = millis();
   // port.write('!'+"9999"+'*');
  }*/

  if(received == true) {
    strokeSensor = (inMSG[1]-'0')*1000+(inMSG[2]-'0')*100+(inMSG[3]-'0')*10+(inMSG[4]-'0');
    //latency = millis() - latencyStart;
    received = false;
  }
  strokeLength = int(map(floor(slider.value()*100),0,100,0,350));//round(200*sin(slider.value()*0.10*PI*t)+300);
  //t=t+0.17;
  if(start == true) {
    if(strokeLength != oldStrokeLength) {
      port.write('!'+nf(strokeLength,4)+'*');
      oldStrokeLength = strokeLength;
    }
    text("Serial: Active",10,110);
  }
  if(start == false) {
    text("Serial: Inactive",10,110);
  }
  fill(0,0,0);
  text("Stroke Value: " + nf(strokeLength,4),10,16);
  text("Sensor Value: " + nf(strokeSensor,4),10,30);
 // text("Latency     : " + nf(latency,4),10,46);

  if(saveData == true) {
    k++;
    if(k>20) {
      time = (millis() - timerStart)/1000;
      saveFile.println(nf(time,5,2)+"  "+nf(strokeLength,4)+" "+nf(strokeSensor,4)+" "+nf(latency,4));
      k=0;
    }
  }
  noStroke();
  fill(0,255,0);
  rect(5,85, 10,10);
  fill(255,0,0);
  rect(235,85, 10,10);
  fill(255,100,0);
  rect(20,85, 10,10);
  guiHitBuffer.beginDraw();
  guiHitBuffer.noStroke();
  guiHitBuffer.fill(0,255,0);
  guiHitBuffer.rect(5,85, 10,10);
  guiHitBuffer.fill(255,0,0);
  guiHitBuffer.rect(235,85, 10,10); 
  guiHitBuffer.fill(255,100,100);
  guiHitBuffer.rect(20,85, 10,10);
  guiHitBuffer.endDraw();
  fill(0,0,0);
  if(exitApp == true) {
    delay(500);
    exit();
  }
}

void mouseReleased() {
  guiEngaged = false;
  guiHitBuffer.beginDraw();
  guiHitBuffer.background(0,0);
  guiHitBuffer.endDraw();
}

void mousePressed() {
  hitColor = guiHitBuffer.get(mouseX,mouseY);
  if(hitColor == color(1) ) {
    guiEngaged = true;
    return;
  }
  if(hitColor == color(0,0)) {
    guiEngaged=false;
    return;
  }
  if(hitColor == color(0,255,0)) {
    start=true;
    //port.write('!'+"9997"+'*');
    if(saveData == false) {
      saveFile = createWriter("data" +nf(savedFiles,2)+".txt");
      saveFile.println("Time StrokeLength StrokeSensor Latency");
      savedFiles++;
      saveData = true;
      timerStart = millis();
      k=0;
      return;
    }
    //port.write('!'+nf(9999,4)+'*');
  }
  if(hitColor == color(255,100,100)) {
    start=false;
   // port.write('!'+"9998"+'*');
    if(saveData ==true) {
      saveFile.flush();
      saveFile.close();
      saveData = false;
    }
    return;
    //port.write('!'+nf(9998,4)+'*');
  }
  if(hitColor == color(255,0,0)) {
    if(saveData ==true) {
      saveFile.flush();
      saveFile.close();
    }
    start = false;
    //port.write('!'+"9998"+'*');
    port.clear();
    port.stop();
    exitApp = true;
    return;
  }
}

void serialEvent(Serial port) {
  if(start == true) {
    inMSG[0] = port.readChar();
    i=0;
    if(inMSG[0] == '!') {
      i++;
      while(inMSG[i-1] != '*') {
        inMSG[i] = port.readChar();
        i++;
      }
    }
    if(i == 6) {
      received = true;
    }
  }
}


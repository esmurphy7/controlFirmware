import processing.serial.*;

Serial port;
int[] msg = new int[8];
int[] sensors = new int[5];
int[] setpoints = new int[5];
boolean received = true;
int timeout = 0;
int timeouts =0;

float t = 0;
float trig;
int i = 0;

PFont font;

boolean[] keys = new boolean[4];

PGraphics guiHitBuffer;
Slider amplitude;
Slider waveNumber;
Slider angularFrequency;

Slider segment1;
Slider segment2;
Slider segment3;
Slider segment4;
Slider segment5;

Button startButton;
Button controlToggle;
Button serialButton;

float Lturn = 1;
float Rturn = 1;

color hitColor=0;

boolean guiActive = false;
boolean amplitudeON = false;
boolean waveNumberON= false;
boolean angularFrequencyON= false;
boolean segment1ON= false;
boolean segment2ON= false;
boolean segment3ON= false;
boolean segment4ON= false;
boolean segment5ON= false;

boolean calibrated = false;

void setup() {
  size(400,600,JAVA2D);
  background(255,255,255);
  guiHitBuffer = createGraphics(400,600,JAVA2D);
  guiHitBuffer.beginDraw();
  guiHitBuffer.background(0);
  guiHitBuffer.endDraw();
  Serial.list();
  port = new Serial(this,Serial.list()[1], 9800); // joey, switch [1] to [0] to run it without an arduino
  println(Serial.list());
  port.bufferUntil(byte(255));

  font = createFont("CourierNewPSMT-14.vlw",14);

  //float x,float y,int w_,int h_,color c_, color ID_ slider
  //float x_, float y_, int h_, int ID_,c, String onString_, String offString_, PFont font_ button
  startButton = new Button(20,20,16,color(150,150,150),color(9),"STOP","START  ",font);
  controlToggle = new Button(20,40,16,color(150,150,150),color(10),"MANUAL ","AUTO    ",font);
  serialButton = new Button(20,330,16,color(150,150,150),color(11),"Serial: OFF","Serial: ON",font);

  //Sine wave stuff
  amplitude = new Slider(200,75,200,20,0,color(1));
  waveNumber = new Slider(200,100,200,20,0,color(2));
  angularFrequency = new Slider(200,125,200,20,0,color(3));

  //Segment stuff
  segment1 = new Slider(200,200,200,20,0,color(4));
  segment2 = new Slider(200,225,200,20,0,color(5));
  segment3 = new Slider(200,250,200,20,0,color(6));
  segment4 = new Slider(200,275,200,20,0,color(7));
  segment5 = new Slider(200,300,200,20,0,color(8));
  segment1.set(100);
  segment2.set(100);
  segment3.set(100);
  segment4.set(100);
  segment5.set(100);

  setpoints[0] = 126;
  setpoints[1] = 126;
  setpoints[2] = 126;
  setpoints[3] = 126;
  setpoints[4] = 126;

  keys[0] = false;
  keys[1] = false;
  keys[2] = false;
  keys[3] = false;
}

void draw() {
  background(255,255,255);
  guiHitBuffer.beginDraw();
  guiHitBuffer.background(0);

  startButton.drawHitbox();
  controlToggle.drawHitbox();
  serialButton.drawHitbox();

  guiHitBuffer.imageMode(CENTER);
  amplitude.drawHitbox();
  waveNumber.drawHitbox();
  angularFrequency.drawHitbox();

  segment1.drawHitbox();
  segment2.drawHitbox();
  segment3.drawHitbox();
  segment4.drawHitbox();
  segment5.drawHitbox();
  guiHitBuffer.imageMode(CORNER);

  guiHitBuffer.endDraw();

  startButton.display();
  controlToggle.display();
  serialButton.display();

  imageMode(CENTER);
  amplitude.display();
  waveNumber.display();
  angularFrequency.display();

  segment1.display();
  segment2.display();
  segment3.display();
  segment4.display();
  segment5.display();
  imageMode(CORNER);

  fill(0,0,0);
  text("Framerate: "+nf(frameRate,2,1),310,10);

  text(nf(253*amplitude.value(),3,1),320,80);
  text(nf(TWO_PI*waveNumber.value(),1,3),320,105);
  text(nf(TWO_PI*angularFrequency.value()*0.01,1,3),320,130);

  text("Amplitude", 29,80);
  text("Wave Number", 5,105);
  text("Ang Freq", 34,130);

  text("Left Mod: "+nf(Lturn,1,2),75,160);
  text("Right Mod:"+nf(Rturn,1,2),200,160);

  text("Segment 1",25,205);
  text("Segment 2",25,230);
  text("Segment 3",25,255);
  text("Segment 4",25,280);
  text("Segment 5",25,305);

  text("Timeouts: "+nf(timeouts,3,0),20,360);

  fill(250,0,0);
  text(nf(sensors[0],3),317,205);
  text(nf(sensors[1],3),317,230);
  text(nf(sensors[2],3),317,255);
  text(nf(sensors[3],3),317,280);
  text(nf(sensors[4],3),317,305);

  fill(0,175,0);
  text(nf(setpoints[0],3),350,205);
  text(nf(setpoints[1],3),350,230);
  text(nf(setpoints[2],3),350,255);
  text(nf(setpoints[3],3),350,280);
  text(nf(setpoints[4],3),350,305);

  if(startButton.getValue() == true) {

    if(keys[0] == true && Lturn > 0.5) {
      Lturn -= Lturn*0.01;
    }
    if(keys[1] == true && Rturn > 0.5) {
      Rturn -= Rturn*0.01;
    }

    if(keys[0] == false && Lturn < 1) {
      Lturn +=0.01;
    }
    if(keys[1] == false && Rturn < 1) {
      Rturn +=0.01;
    }

    if(controlToggle.getValue() == true) {
      for(int i=0;i<5;i++) {
        trig = sin(TWO_PI*waveNumber.value()*i+TWO_PI*angularFrequency.value()*t*0.01);
        if(trig < 0) {
          setpoints[i] = floor(127+126*Rturn*amplitude.value()*trig);
        }
        else {
          setpoints[i] = floor(127+126*Lturn*amplitude.value()*trig);
        }
        if(keys[2] == true) {
          t+=0.17;
        }
      }
      if(guiActive == false) {
        if(serialButton.getValue() == true) {
          if(received ==true) {
            port.write(byte(254));
            port.write(byte(setpoints[0]));
            port.write(byte(setpoints[1]));
            port.write(byte(setpoints[2]));
            port.write(byte(setpoints[3]));
            port.write(byte(setpoints[4]));
            port.write(byte(255));
            timeout = millis();
            received = false;
          }
          else {
            if(millis()-timeout > 500) {
              timeouts++;
              timeout = millis();
            }
          }
        }
        segment1.set(map(setpoints[0],0,253,0,200));
        segment2.set(map(setpoints[1],0,253,0,200));         
        segment3.set(map(setpoints[2],0,253,0,200));          
        segment4.set(map(setpoints[3],0,253,0,200));
        segment5.set(map(setpoints[4],0,253,0,200));
      }
    }
    else {
      if(guiActive == true) {
        setpoints[0] = floor(segment1.value()*253);
        setpoints[1] = floor(segment2.value()*253);
        setpoints[2] = floor(segment3.value()*253);
        setpoints[3] = floor(segment4.value()*253);
        setpoints[4] = floor(segment5.value()*253);
        if(serialButton.getValue() == true) {
          if(received ==true) {
            port.write(byte(254));
            port.write(byte(setpoints[0]));
            port.write(byte(setpoints[1]));
            port.write(byte(setpoints[2]));
            port.write(byte(setpoints[3]));
            port.write(byte(setpoints[4]));
            port.write(byte(255));
            timeout = millis();
            received = false;
          }
          else {
            if(millis()-timeout > 500) {
              timeouts++;
              timeout = millis();
            }
          }
        }
      }
    }
  }
}

void mousePressed() {
  hitColor = guiHitBuffer.get(mouseX,mouseY);
  if(hitColor == color(9)) {
    startButton.toggle();
  }
  if(hitColor == color(10)) {
    controlToggle.toggle();
  }
  if(hitColor == color(11)) {
    serialButton.toggle();
  }
  if(hitColor == color(0)) {
    guiActive = false;
  }
  if(hitColor == color(1)) {
    amplitudeON = true;
    guiActive = true;
  }
  if(hitColor == color(2)) {
    waveNumberON = true;
    guiActive = true;
  }
  if(hitColor == color(3)) {
    angularFrequencyON = true;
    guiActive = true;
  }
  if(controlToggle.getValue() == false) {
    if(hitColor == color(4)) {
      segment1ON = true;
      guiActive = true;
    }
    if(hitColor == color(5)) {
      segment2ON = true;
      guiActive = true;
    }
    if(hitColor == color(6)) {
      segment3ON = true;
      guiActive = true;
    }
    if(hitColor == color(7)) {
      segment4ON = true;
      guiActive = true;
    }
    if(hitColor == color(8)) {
      segment5ON = true;
      guiActive = true;
    }
  }
}

void mouseDragged() {
  if(guiActive == true) {
    if(amplitudeON == true) {
      amplitude.update();
    }
    if(waveNumberON == true) {
      waveNumber.update();
    }
    if(angularFrequencyON == true) {
      angularFrequency.update();
    }
    if(segment1ON == true) {
      segment1.update();
    }
    if(segment2ON == true) {
      segment2.update();
    }
    if(segment3ON == true) {
      segment3.update();
    }
    if(segment4ON == true) {
      segment4.update();
    }
    if(segment5ON == true) {
      segment5.update();
    }
  }
}

void mouseReleased() {
  if(guiActive == true) {
    guiActive = false;
    amplitudeON = false;
    waveNumberON = false;
    angularFrequencyON= false;
    segment1ON = false;
    segment2ON = false;
    segment3ON = false;
    segment4ON = false;
    segment5ON = false;
  }
}

void keyPressed() {
  if(keyCode == LEFT) {
    keys[0] = true;
  }
  if(keyCode == RIGHT) {
    keys[1] = true;
  } 
  if(keyCode == UP) {
    keys[2] = true;
  } 
  if(keyCode == DOWN) {
    keys[3] = true;
  }
}

void keyReleased() {
  if(keyCode == LEFT) {
    keys[0] = false;
  }
  if(keyCode == RIGHT) {
    keys[1] = false;
  } 
  if(keyCode == UP) {
    keys[2] = false;
  } 
  if(keyCode == DOWN) {
    keys[3] = false;
  }
}

void serialEvent(Serial port) {
  msg[0] = port.read();
  i=0;
  if(msg[0] == 254) {
    i++;
    while(msg[i-1] != 255) {
      msg[i] = port.read();
      i++;
      if(i > 7) {
        break;
      }
    }
  }
  if(i == 7) {
    sensors[0] = msg[1];
    sensors[1] = msg[2];
    sensors[2] = msg[3];
    sensors[3] = msg[4];
    sensors[4] = msg[5];
    received = true;
  }
}


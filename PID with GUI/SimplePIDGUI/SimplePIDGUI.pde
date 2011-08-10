//import processing.core.*;
import processing.serial.*;
import controlP5.*;

//public class MyProcessingSketch extends PApplet {

ControlP5 controlP5;

// serial port
Serial myPort;

// text field and button for constants
// product constant, derivative constant, integral constant
Textfield tKp, tKd, tKi;

// position
Slider sPosition;
// period
Slider sPeriod;
//refreash rate for setpoints
Slider sRefresh;
// choosing shape of wave
RadioButton wave;

// start button
Toggle tStart;
boolean running = false;

// gradian
double gradian = 0;

// how long befor updating setpoint
int updatePhase = 0;

// charts
Chart chart1;
Textfield chart1a;
Chart chart2;
Textfield chart2a;
Chart chart3;
Textfield chart3a;

// data arrays for charts
int chartLength = 200;
float[] sentPositions = new float[chartLength];
float[] receivePositions = new float[chartLength];


public void setup() {
  size(400, 600);
  smooth();
  frameRate(30);

  controlP5 = new ControlP5(this);

  // setup serial
  // List all the available serial ports
  println(Serial.list());

  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[0], 115200);

  CColor cGreen = new CColor();
  cGreen.setBackground(color(64, 128, 64));
  cGreen.setForeground(color(0, 225, 0));
  cGreen.setActive(color(128, 255, 128));
  cGreen.setCaptionLabel(color(0));

  CColor cWhite = new CColor();
  cWhite.setBackground(color(255));
  cWhite.setForeground(color(128));
  cWhite.setActive(color(255));
  cWhite.setCaptionLabel(color(0));
  cWhite.setValueLabel(color(0));

  CColor cRed = new CColor();
  cRed.setBackground(color(128, 64, 64));
  cRed.setForeground(color(255, 0, 0));
  cRed.setActive(color(255, 128, 128));
  cRed.setCaptionLabel(color(0));

  // text field to set constants
  tKp = controlP5.addTextfield("Kp", 20, 40, 60, 20);
  tKp.setColor(cWhite);
  tKp.setAutoClear(false);
  tKp.setText("50");

  tKd = controlP5.addTextfield("Kd", 90, 40, 60, 20);
  tKd.setColor(cWhite);
  tKd.setAutoClear(false);
  tKd.setText("0");

  tKi = controlP5.addTextfield("Ki", 160, 40, 60, 20);
  tKi.setColor(cWhite);
  tKi.setAutoClear(false);
  tKi.setText("0");


  controlP5.addBang("setConstants", 230, 40, 60, 20).setColor(cGreen);
  
  controlP5.addBang("Calibrate", 320, 40, 40, 20).setColor(cRed);
  
  // start toggle
  tStart = controlP5.addToggle("Start", false, 20, 100, 270, 55);
  tStart.setColor(cRed);

  // variables that determine set points
  
  wave = controlP5.addRadioButton("radio", 20, 210);
  wave.setItemsPerRow(5);
  wave.setSpacingColumn(50);
  wave.setColorLabel(color(0));
  wave.addItem("manual", 0);
  wave.addItem("sine", 1);
  wave.addItem("triangle", 2);
  wave.addItem("rectangle", 3);
  wave.activate(0);

  sPosition = controlP5.addSlider("position", 0, 255, 127, 20, 240, 300, 23);
  sPosition.setSliderMode(Slider.FLEXIBLE);
  sPosition.captionLabel().setColor(color(0));

  sPeriod = controlP5.addSlider("period", 1, 10, 4, 20, 270, 300, 15);
  sPeriod.setColorCaptionLabel(color(0));
  
  sRefresh = controlP5.addSlider("refresh rate", 0, 20, 0, 20, 300, 300, 15);
  sRefresh.setColorCaptionLabel(color(0));
  
  // charts
  chart1 = controlP5.addChart("posChart", 20, 340, 300, 128);
  chart1.setColorBackground(color(128));
  chart1.addDataSet().getColor().setForeground(color(255));
  chart1a = controlP5.addTextfield("SPOS", 320, 345, 80, 20);
  chart1a.setColor(cWhite);
  chart1a.setAutoClear(false);
}


public void draw() {
  background(192);
  // round off ranges to the nearest integer
  // and keep within range limits
  updateRanges();

  if (tStart.value() != 0) {
    // calculate new set points for a wave
    if (wave.value() != 0) {
      updatePosition();
    }

    if (!running) {
      //tell arduino to start
      myPort.write('1');
      println('1');
      running = true;
    }
    // send and receive positions
    communicate();
    //update charts
    updateChart();
  }
  if (tStart.value() == 0) {
    if (running) {
      //tell arduino to stop
      myPort.write('0');
      println('0');
      //myPort.stop();
      running = false;
    }
  }
  
  delay(100);
}

void updatePosition() {
  // update gradian
  gradian += 10 / sPeriod.value();
  if (gradian>400) {
    gradian -= 400;
  }
  
  int high = 240;
  
  int low = 15;
  
  updatePhase++;
  if(updatePhase>sRefresh.value()){
    updatePhase = 0;
  }
  if(updatePhase == 0){
    switch((int)wave.value()) {
    case 1:
      // sine wave
      sPosition.setValue((float)((high-low)*(Math.sin(2*Math.PI*gradian/400)+1)/2 + low));
      break;
    case 2:
      // triangle wave
      if (gradian<100) {
        sPosition.setValue((float)((gradian+100)*(high-low)/200+low));
      }
      else if (gradian<300) {
        sPosition.setValue((float)(gradian-100)*(low-high)/200+high);
      }
      else {
        sPosition.setValue((float)(gradian-300)*(high-low)/200+low);
      }
      break;
    case 3:
      // square wave
      if (gradian<200) {
        sPosition.setValue(high);
      }
      else {
        sPosition.setValue(low);
      }
      break;
    }
  }
  // round position to nearest whole number
  sPosition.setValue(Math.round(sPosition.value()));
}

int prevPosition=0;

void communicate() {
  
  if((int)wave.value()==0){
    if(prevPosition != (int)sPosition.value()){
      myPort.write('s');
      myPort.write((byte)sPosition.value());
      //myPort.write((byte)sRefresh.value());
      println("s "+ (int)sPosition.value() +" " + (int)sRefresh.value());
      prevPosition = (int)sPosition.value();
    }
  }
  else if(updatePhase == 0){
    //send set position
    myPort.write('s');
    myPort.write((byte)sPosition.value());
    myPort.write((byte)sRefresh.value());
    println("s "+ (int)sPosition.value() +" " + (int)sRefresh.value());
  }
  
  //update sent position on charts
  System.arraycopy(sentPositions, 0, sentPositions, 1, chartLength-1);
  sentPositions[0] = (float)sPosition.value();

  int inMessage = 0;
  //get position
  if (myPort.available()>0) {
    inMessage = myPort.read();
  }
  chart1a.setText(Integer.toString(inMessage));
  //update received Position 
  System.arraycopy(receivePositions, 0, receivePositions, 1, chartLength-1);
  receivePositions[0] = (float)inMessage;
  myPort.clear();
}

void updateChart() {
  float[] tempArray = new float[chartLength];

  //scale data and draw them
  //sent positions
  for (int i=0;i<chartLength;i++) {
    tempArray[i] = sentPositions[i] / 255 * 128;
    if (tempArray[i]<0) {
      tempArray[i] = 0;
    }
    if (tempArray[i]>128) {
      tempArray[i] = 128;
    }
  }
  chart1.updateData(0, tempArray);

  //received position
  for (int i=0;i<chartLength;i++) {
    tempArray[i] = receivePositions[i] / 255 * 128;
    if (tempArray[i]<0) {
      tempArray[i] = 0;
    }
    if (tempArray[i]>128) {
      tempArray[i] = 128;
    }
  }
  chart1.updateData(1, tempArray);
}

void updateRanges() {

  // round positions
  sPosition.setValue(Math.round(sPosition.value()));
  sRefresh.setValue(Math.round(sRefresh.value()));
}

public void setConstants() {
  Textfield input = null;
  
  myPort.write('k');
  print("k ");
  
  for (int i = 0; i < 3; i++) {
    //loop through each text field input
    switch (i) {
    case 0:
      input = tKp;
      break;
    case 1:
      input = tKd;
      break;
    case 2:
      input = tKi;
      break;
    }
    
    try {
      //try to get usable data from text field 
      int value = Integer.parseInt(input.getText());
      if(value < 0){
        value = 0;
      }
      else if(value>255){
        value = 255;
      }
      input.setText(Integer.toString(value));
      myPort.write((byte)value);
      print(value + " ");
    }
    catch (Exception e) {
      input.setText("0");
      myPort.write((byte)0);
      print("0");
    }
  }
  
  print("\n");
}

public void Calibrate() {
  myPort.write('c');
  println("c");
  tStart.setState(false);
}

//}


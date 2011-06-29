/*
Snake Controls Simulation 2.0
move mouse up or down to increase or decrease number of bends
move mouse left or right to tun that direction
press up or down to to change width of movments(size constant)
*/
final int NumberOfSegments = 30;
final int SegmentLength = 10;
final float Range = QUARTER_PI;

float[] angle = new float[NumberOfSegments];

float headAngle = 0;
float headTurn = 0;
float headAmplitude = 0.2;
float headRadian = 0;
float frequency = .4;
float sizeConstant = 0.6;

float[] xArray = new float[NumberOfSegments];
float[] yArray = new float[NumberOfSegments];
float xCenter = 0;
float yCenter = 0;
float rotationCenter = 0;

void setup(){
  size(600, 600);
  smooth();
  
  strokeWeight(10);
}

void draw(){
  background(226);
  
  //next radian
  headRadian += frequency;
  if(headRadian > TWO_PI){
    headRadian -= TWO_PI;
  }
  if(headRadian < 0){
    headRadian += TWO_PI;
  }
  
  headTurn = ((float)mouseX/width-0.5)* QUARTER_PI/4;
  frequency = ((float)(height-mouseY)/height) * 0.4+0.2;
  headAmplitude = sizeConstant*frequency;
  
  //next positions
  //headAngle = (((float)mouseX/width)-0.5)*Range;
  headAngle = headAmplitude*sin(headRadian) + headTurn;
  headAngle = constrain(headAngle, -Range, Range);
  
  //move angles down snake adding headAngle to front
  propergate(headAngle,0.0,10.0);
  
  translate(300,300);
  //draw invisable snake to find centers
  noStroke();
  extendFwd(0,0,0,0);
  findCenter();
  //translate coordinates and redraw
  stroke(10);
  rotate(rotationCenter);
  translate(-xCenter,-yCenter);
  extendFwd(0,0,0,0);
  //reset coordinate
  resetMatrix();
  
  delay(100);
}

void extendFwd(int segment,float x,float y,float currentAngle){
  
  float nextX = x+SegmentLength*sin(currentAngle+angle[segment]);
  float nextY = y+SegmentLength*cos(currentAngle+angle[segment]);
  
  line(x, y, nextX, nextY);
  
  xArray[segment] = x;
  yArray[segment] = y;
  
  if(segment+1 < NumberOfSegments){
    extendFwd(segment+1,nextX,nextY,currentAngle+angle[segment]);
  }
}

void propergate(float newAngle, float newTurn, float newAmplitude){
  for(int i= NumberOfSegments-1;i>0;i--){
    angle[i] = angle[i-1];
  }
  angle[0] = newAngle;
}

void findCenter(){
  float totalX = 0;
  float totalY = 0;
  float totalRotation = 0;
  
  for(int i=0; i<NumberOfSegments;i++){
    totalX += xArray[i];
    totalY += yArray[i];
    if(yArray[i] != 0){
      totalRotation += atan2(xArray[i],yArray[i]);
    }
  }
  
  xCenter = totalX/NumberOfSegments;
  yCenter = totalY/NumberOfSegments;
  rotationCenter = totalRotation/NumberOfSegments;
}

void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      sizeConstant+=0.01;
    } else if (keyCode == DOWN) {
      sizeConstant-=0.01;
    } 
  }
  sizeConstant = constrain(sizeConstant, 0.2, 1);
  println(sizeConstant);
}

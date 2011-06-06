class Button {
  float locx,locy;
  int w,h; 
  boolean value;
  int ID;
  String onString;
  String offString;
  PFont font;
  color c;

  PGraphics tempBuffer;
  PImage buttonON;
  PImage buttonOFF;
  PImage hitBox;

  Button(float x_, float y_, int h_,color c_, int ID_, String onString_, String offString_, PFont font_) { //width in pixels
    locx=x_;
    locy=y_;
    c = c_;
    onString = onString_;
    offString = offString_;
    h=h_;
    ID = ID_;
    value = false;
    font = font_;

    w = onString.length()*7+4;
    if(offString.length()*7+4 > w) {
      w= offString.length()*7+4;
    }

    tempBuffer = createGraphics(w,h,JAVA2D);
    tempBuffer.beginDraw();
    tempBuffer.background(c+color(225,225,225));
    tempBuffer.fill(0,0,0);
    tempBuffer.textFont(font);
    tempBuffer.text(onString,2,h-2);
    tempBuffer.endDraw();
    buttonON = tempBuffer;

    tempBuffer = createGraphics(w,h,JAVA2D);
    tempBuffer.beginDraw();
    tempBuffer.background(c);
    tempBuffer.fill(0,0,0);
    tempBuffer.textFont(font);
    tempBuffer.text(offString,2,h-2);
    tempBuffer.endDraw();
    buttonOFF = tempBuffer;

    tempBuffer = createGraphics(w,h,JAVA2D);
    tempBuffer.beginDraw();
    tempBuffer.background(ID);
    tempBuffer.endDraw();
    hitBox = tempBuffer;
  }

  void display() {

    if(value == false) {
      image(buttonOFF,locx,locy);
    }
    if(value == true) {
      image(buttonON,locx,locy);
    }

    return;
  }
  void drawHitbox(){
    guiHitBuffer.image(hitBox,locx,locy);
    return;
  }

  boolean getValue() {
    return value;
  }

  int getIntValue() {
    int intVal;
    if(value==true) {
      intVal = 1;
    }
    else {
      intVal = 0;
    }
    return intVal;
  }

  void toggle() {
    if(value == true) {
      value = false;
      return;
    }
    if(value == false) {
      value = true;
      return;
    }
  }

  void toggleOFF() {
    value = false;
  }
  void toggleON() {
    value = true;
  }
}


class Slider {
  float locx,locy,locz; //specified from center of slider
  int w,h; //the width is of the slider not it's background, backgound willbe 20 pixels wider
  float value;
  color c;
  color ID;
  PGraphics tempBuffer;
  PImage slider;
  PImage sliderBack;
  PImage hitBox;

  Slider(float x,float y,int w_,int h_,color c_, color ID_) { //z usually 0
    locx=x;
    locy=y;
    w=w_;
    h=h_;
    c=c_;
    value =0;
    ID = ID_;

    tempBuffer = createGraphics(w+w/10+1,h+1,P2D);
    tempBuffer.beginDraw();
    tempBuffer.background(0,0);
    tempBuffer.stroke(c); //make background
    tempBuffer.fill(255,255,255);
    tempBuffer.rect(0,0,w+w/10,h);
    tempBuffer.strokeWeight(3);
    tempBuffer.stroke(red(c),green(c),blue(c),255/2);
    tempBuffer.line(w/20,h/2,w+w/20,h/2);
    tempBuffer.endDraw();
    sliderBack = tempBuffer;

    tempBuffer = createGraphics(int(h*0.8),int(h*0.8),P2D); //make slider
    tempBuffer.beginDraw();
    tempBuffer.background(0,0);
    tempBuffer.noStroke();
    tempBuffer.fill(c);
    //tempBuffer=bezierRect(0, 0, int(w/20),int(w/20), -10, -10);
    tempBuffer.rect(0,0,int(h*0.8),int(h*0.8));
    tempBuffer.endDraw();
    slider = tempBuffer;

    tempBuffer = createGraphics(int(h*0.8),int(h*0.8),P2D); //make hitBox
    tempBuffer.beginDraw();
    tempBuffer.background(0,0);
    tempBuffer.noStroke();
    tempBuffer.fill(ID);
    tempBuffer.rect(0,0,int(h*0.8),int(h*0.8));
    tempBuffer.endDraw();
    hitBox = tempBuffer;
  }

  void display() {
    //imageMode(CENTER);
    //pushMatrix();
    //translate(locx,locy);
    tint(255,255*0.75);
    image(sliderBack,locx,locy);
    noTint();
    image(slider,locx-w/2+w*(value/w), locy);
    //popMatrix();
    //imageMode(CORNER);
    /*
    guiHitBuffer.beginDraw();
     guiHitBuffer.imageMode(CENTER);
     guiHitBuffer.pushMatrix();
     guiHitBuffer.translate(locx,locy);
     guiHitBuffer.image(hitBox,-w/2+(value), 0);
     guiHitBuffer.popMatrix();
     guiHitBuffer.imageMode(CORNER);
     guiHitBuffer.endDraw();*/
    return;
  }

  void drawHitbox() {
    //guiHitBuffer.translate(locx,locy);
    guiHitBuffer.image(hitBox,locx-w/2+(value), locy);
    return;
  }

  void update() { // only while mosue button being held
    if(value<=w && value >=0) {
      value+=mouseX-pmouseX;
    }
    if(value>w) {
      value = w;
    }
    if(value<0) {
      value=0;
    }
    return;
  }

  float value() {
    return value/w;
  }
  void set(float value_) {
    value = value_;
    return;
  }
}


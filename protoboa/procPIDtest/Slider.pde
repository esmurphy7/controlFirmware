class Slider {
  float locx,locy,locz; //specified from center of slider
  int w,h; //the width is of the slider not it's background, backgound willbe 20 pixels wider
  float value;
  color c;
  PGraphics tempBuffer;
  PImage slider;
  PImage sliderBack;
  PImage hitBox;

  Slider(float x,float y,float z,int w_,int h_,color c_) { //z usually 0
    locx=x;
    locy=y;
    locz=z;
    w=w_;
    h=h_;
    c=c_;
    value =0;

    tempBuffer = createGraphics(w+w/10+1,h+1,P2D);
    tempBuffer.beginDraw();
    tempBuffer.background(0,0);
    tempBuffer.stroke(c); //make background
    tempBuffer.fill(255,255,255);
    //tempBuffer=bezierRect(0,0, w+20, h, -10, -10);
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
    tempBuffer.fill(1);
    //tempBuffer=bezierRect(0, 0, int(w/20),int(w/20), -10, -10);
    tempBuffer.rect(0,0,int(h*0.8),int(h*0.8));
    tempBuffer.endDraw();
    hitBox = tempBuffer;
  }

  void display() {
    imageMode(CENTER);
    pushMatrix();
    translate(locx,locy,locz);
    tint(255,255*0.75);
    image(sliderBack,0,0);
    noTint();
    image(slider,-w/2+w*(value/w), 0);
    popMatrix();

    guiHitBuffer.beginDraw();
    guiHitBuffer.imageMode(CENTER);
    guiHitBuffer.pushMatrix();
    guiHitBuffer.translate(locx,locy);
    guiHitBuffer.image(hitBox,-w/2+w*(value/w), 0);
    guiHitBuffer.popMatrix();
    guiHitBuffer.endDraw();
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
  }

  float value() {
    return value/w;
  }
  
  void setValue(float value_){
   value = value_; 
  }
}


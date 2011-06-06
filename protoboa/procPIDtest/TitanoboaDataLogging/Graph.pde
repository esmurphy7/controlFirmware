class Graph {
  float locx,locy;
  int w,h;
  String title,xlabel,ylabel;
  float xdata[];//these will simply point to the actual data
  float ydata[];

  PGraphics tempBuffer;
  PImage graphBack;
  Phrase tempPhrase;
  int margin;

  int dataLength;
  float ydataMax;
  float xdataMax;

  Graph(float xdata_[],float ydata_[], int w_, int h_,
  String title_,String xlabel_,String ylabel_,float xdataMax_,float ydataMax_) {
    locx=0;
    locy=0;
    w=w_;
    h=h_;
    xdata = xdata_;
    ydata = ydata_;
    ydataMax=ydataMax_;
    xdataMax=xdataMax_;
    dataLength = ydata.length;
    title = title_;
    xlabel = xlabel_;
    ylabel = ylabel_;
    tempPhrase = new Phrase(" ","CourierNewPSMT-12.vlw");
    margin = int(tempPhrase.getSize())+6;
    tempBuffer = createGraphics(w+2*margin,h+2*margin,P2D);
    //make background
    tempBuffer.beginDraw();
    tempBuffer.background(255);
    tempBuffer.stroke(0,0,0);
    tempBuffer.line(margin,margin,margin,h+margin); //y
    tempBuffer.line(margin,h+margin,w+margin,h+margin); //x
    graphBack = tempBuffer;
  }


  void display(float x,float y) {
    locx=x;
    locy=y;
    pushMatrix();
    translate(locx,locy,0);
    tint(255,255*0.9);
    imageMode(CORNER);
    image(graphBack,0,0);
    noTint();
    textAlign(CENTER);
    fill(255,255,255);
    tempPhrase.updateString(xlabel);
    tempPhrase.display(w/2+margin+3,h+2*margin-3,0);
    tempPhrase.updateString(title);
    tempPhrase.display(w/2+margin+3,margin-3,0);
    translate(margin-3,h/2+margin-3,0);
    rotate(-PI/2);
    tempPhrase.updateString(ylabel);
    tempPhrase.display(0,0,0);
    textAlign(LEFT);
    rotate(PI/2);
    //translate(margin,h+margin);
    strokeWeight(0.5);
    for(int i=0;i<dataLength;i++) {
      stroke(255,0,0);
      line(3+xdata[i]*w/xdataMax, //x1
      -abs(h*ydata[i]/ydataMax)+h/2+3, //y1
      3+xdata[i]*w/xdataMax, //x2
      h/2+3); //y2
    }
    strokeWeight(1);
    popMatrix();
  }
}


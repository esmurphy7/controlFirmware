class Textfield {
  float locx,locy;
  float w;
  int ID;
  int digit;
  String tempString;
  String displayText;
  char[] inText;
  int savedValue;
  //Phrase phrase; will just use the courier12 phrase for this

  Textfield(int w_, int i_) { //w in characters

    w=w_*14;
    h=14;
    ID=i_;
    displayText = "";
    digit =0;
    savedValue =0;
    inText = new char[w_];
    inText[0] ='0';
    inText[1]='0';
    inText[2]='0';
    inText[3]='0';
  }

  void display(float locx_, float locy_) {
    locx = locx_;
    locy=locy_;
    pushMatrix();
    translate(locx,locy);
    fill(240,240,240);
    if(guiActiveArray[ID] == true) {
      fill (200,200,200);
    }
    rect(0,0, w+4,h+4);
    fill(0,0,0);
    courier14.updateString(displayText);
    if(guiActiveArray[ID] == false) {
     displayText = nf(savedValue,4);
     }
    courier14.display(2,2+14);
    popMatrix();

    hitBuffer.beginDraw();
    hitBuffer.pushMatrix();
    hitBuffer.translate(locx+width/2,locy+height/2);
    hitBuffer.fill(ID);
    hitBuffer.noStroke();
    hitBuffer.rect(0,0, w+4,h+4);
    hitBuffer.popMatrix();
    hitBuffer.endDraw();
  }

  void update() {
    if(digit<4) {
      inText[digit] = key;
      digit++;
      displayText = new String(inText);
    }
    if(key == ENTER) {
      savedValue = parseInt(displayText,10);
      guiActiveArray[ID] = false;
      guiActive = false;
      digit=0;
      inText[0] ='0';
      inText[1]='0';
      inText[2]='0';
      inText[3]='0';
    }
  }

  int getValue() {
    return savedValue;
  }
}


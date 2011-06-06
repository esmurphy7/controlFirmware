class Phrase {
  float locationX;
  float locationY;
  float locationZ;

  float theta;
  float phi;

  String phrase;
  float fontSize; // text height, ALWAYS PICK INTEGER VALUES
  color c;
  PFont font;
  float textLength;

  Phrase(String phrase_,String fontType_, color c_, float fontSize_) {
    locationX=0; 
    locationY=0;
    locationZ=0;

    theta =0;
    phi =0;

    c=c_;
    phrase = phrase_;
    fontSize=fontSize_;
    font = loadFont(fontType_);
    textLength = textWidth(phrase);
  }

  Phrase(String phrase_,String fontType_,color c_) {
    int i;
    String tempString;

    locationX=0; 
    locationY=0;
    locationZ=0;

    theta =0;
    phi =0;

    c=c_;
    phrase = phrase_;

    i=fontType_.lastIndexOf('-')+1;
    tempString = fontType_.substring(fontType_.indexOf('-')+1,fontType_.indexOf('.'));
    fontSize=stringTOint(tempString);
    font = loadFont(fontType_);
    textLength = textWidth(phrase);
  }


  Phrase(String phrase_,String fontType_) {
    int i;
    String tempString;

    locationX=0; 
    locationY=0;
    locationZ=0;

    theta =0;
    phi =0;

    c= color(0,0,0);
    phrase = phrase_;

    i=fontType_.lastIndexOf('.')+1;
    tempString = fontType_.substring(fontType_.indexOf('-')+1,fontType_.indexOf('.'));
    fontSize=stringTOint(tempString);
    font = loadFont(fontType_);
    textLength = textWidth(phrase);
  }

  void display(float locationX_,float locationY_,float locationZ_,
  float theta_, float phi_) {
    locationX=locationX_; 
    locationY=locationY_;
    locationZ=locationZ_;
    theta =theta_;
    phi =phi_;

    pushMatrix();    
    rotateY(phi);
    rotateZ(theta);
    fill(c);
    textFont(font,fontSize);
    translate(locationX,locationY,locationZ);
    text(phrase,0,0,0);
    popMatrix();
  }

  void display(float locationX_,float locationY_,float locationZ_) {
    locationX=locationX_; 
    locationY=locationY_;
    locationZ=locationZ_;
    theta =0;
    phi =0;

    pushMatrix();
    rotateY(phi);
    rotateZ(theta);
    fill(c);
    textFont(font,fontSize);
    translate(locationX,locationY,locationZ);
    text(phrase,0,0,0);
    popMatrix();
  }

  void display(float locationX_,float locationY_) {
    locationX=locationX_; 
    locationY=locationY_;
    fill(c);
    textFont(font,fontSize);
    text(phrase,locationX,locationY);
  }

  void display(float locationX_,float locationY_, PGraphics buffer) {
    locationX=locationX_; 
    locationY=locationY_;

    buffer.pushMatrix();
    buffer.fill(c);
    buffer.textFont(font,fontSize);
    buffer.translate(locationX,locationY);
    buffer.text(phrase,0,0,0);
    buffer.popMatrix();
  }

  void updateLocation(float locationX_,float locationY_,float locationZ_) {
    locationX=locationX_; 
    locationY=locationY_;
    locationZ=locationZ_;
  }
  void updateLocation(float locationX_,float locationY_,float locationZ_,
  float theta_, float phi_) {
    locationX=locationX_; 
    locationY=locationY_;
    locationZ=locationZ_;
    theta =theta_;
    phi =phi_;
  }
  void updateString(String phrase_) {
    phrase = phrase_;
  }
  void updatePhrase(String phrase_, PFont font_) {
    phrase = phrase_;
    font = font_;
  }

  void updateColor(color c_) {
    c = c_;
  }

  void updateSize(int size_) {
    fontSize = size_;
  }

  float getX() {
    return locationX;
  }
  float getY() {
    return locationY;
  }
  float getZ() {
    return locationZ;
  }
  float getTheta() {
    return theta;
  }
  float getPhi() {
    return phi;
  }
  float getLength() {
    return textLength;
  }
  float getSize() {
    return fontSize;
  }
  String getString() {
    return(phrase);
  }
}


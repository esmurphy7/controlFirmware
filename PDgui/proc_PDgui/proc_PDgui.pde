import processing.core.*;
import processing.serial.*;
import controlP5.*;

//public class MyProcessingSketch extends PApplet {
	
	ControlP5 controlP5;
	
	// serial port
	Serial myPort;
	
	// text field and button for constants
	// max speed, max acceleration, product constant, derivative constant
	Textfield tSp,tAc,tKp,tKd;
	
	// limit set points to send
	Range rRange;
	//range for tests
	Range rAmplitude;
	// position
	Slider sPosition;
	// period
	Slider sPeriod;
	// choosing shape of wave
	RadioButton wave;
	
	// start button
	Toggle tStart;
	boolean running = false;
	
	// gradian
	double gradian = 0;
	
	// charts
	Chart chart1, chart2, chart3;
	Textfield chart1a, chart2a, chart3a;
	
	// data arrays for charts
	int chartLength = 200;
	float[] sentPositions = new float[chartLength];
	float[] receivePositions = new float[chartLength];
	float[] PDvalues = new float[chartLength];
	float[] derivatives = new float[chartLength];
	
	
	public void setup() {
		size(400, 740);
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
		tSp = controlP5.addTextfield("Speed", 20, 40, 60, 20);
		tSp.setColor(cWhite);
		tSp.setAutoClear(false);
		tSp.setText("255");
		
		tAc = controlP5.addTextfield("Acceleration", 90, 40, 60, 20);
		tAc.setColor(cWhite);
		tAc.setAutoClear(false);
		tAc.setText("128");
		
		tKp = controlP5.addTextfield("Kp", 160, 40, 60, 20);
		tKp.setColor(cWhite);
		tKp.setAutoClear(false);
		tKp.setText("128");
		
		tKd = controlP5.addTextfield("Kd", 230, 40, 60, 20);
		tKd.setColor(cWhite);
		tKd.setAutoClear(false);
		tKd.setText("0");
		
		controlP5.addBang("setConstants", 310, 40, 60, 20).setColor(cGreen);
		
		// start toggle
		tStart = controlP5.addToggle("Start", false, 20, 100, 270, 55);
		tStart.setColor(cRed);
		
		// variables that determine set points
		rRange = controlP5.addRange("range",0,1023,150,300,20,180,300,15);
		rRange.setColorCaptionLabel(color(0));
		
		wave = controlP5.addRadioButton("radio", 20, 210);
		wave.setItemsPerRow(5);
		wave.setSpacingColumn(50);
		wave.setColorLabel(color(0));
		wave.addItem("manual", 0);
		wave.addItem("sine", 1);
		wave.addItem("triangle", 2);
		wave.addItem("rectangle", 3);
		wave.activate(0);
		
		rAmplitude = controlP5.addRange("amplitude",0,1023,200,250,20,240,300,15);
		rAmplitude.setColorCaptionLabel(color(0));
		
		sPosition = controlP5.addSlider("position", 0, 1023, 225, 20, 270, 300, 23);
		sPosition.setSliderMode(Slider.FLEXIBLE);
		sPosition.captionLabel().setColor(color(0));
		
		sPeriod = controlP5.addSlider("period",1,10,4,20,310,300,15);
		sPeriod.setColorCaptionLabel(color(0));
		
		// charts
		chart1 = controlP5.addChart("posChart", 20, 340, 300, 128);
		chart1.setColorBackground(color(128));
		chart1.addDataSet().getColor().setForeground(color(255));
		chart1a = controlP5.addTextfield("SPOS", 320, 345, 80, 20);
		chart1a.setColor(cWhite);
		chart1a.setAutoClear(false);
		
		chart2 = controlP5.addChart("derChart", 20, 470, 300, 128);
		chart2.setColorBackground(color(128));
		chart2.getDataSet().getColor().setForeground(color(225,0,225));
		chart2a = controlP5.addTextfield("SDRV", 320, 475, 80, 20);
		chart2a.setColor(cWhite);
		chart2a.setAutoClear(false);
		
		chart3 = controlP5.addChart("pdChart", 20, 600, 300, 128);
		chart3.setColorBackground(color(128));
		chart3.getDataSet().getColor().setForeground(color(255,225,0));
		chart3a = controlP5.addTextfield("SPDV", 320, 605, 80, 20);
		chart3a.setColor(cWhite);
		chart3a.setAutoClear(false);
	}
	
	
	public void draw() {
		background(192);
		// round off ranges to the nearest integer
		// and keep within range limits
		updateRanges();
		
		if(tStart.value() != 0){
			// calculate new set points for a wave
			if(wave.value() != 0){
				updatePosition();
			}
			
			if(!running){
				myPort.write("STAT0000"+'!');
				running = true;
			}
			// send and receive positions
			communicate();
			//update charts
			updateChart();
		}
		if(tStart.value() == 0){
			if(running){
				myPort.write("STOP0000" +'!');
				//myPort.stop();
				running = false;
			}
		}
	}

	void updatePosition(){
		// update gradian
		gradian += 10 / sPeriod.value();
		if(gradian>400){
			gradian -= 400;
		}
		
		float high = rAmplitude.highValue();
		float low = rAmplitude.lowValue();
		
		
		switch((int)wave.value()){
		case 1:
			// sine wave
			sPosition.setValue((float)((high-low)*(Math.sin(2*Math.PI*gradian/400)+1)/2 + low));
			break;
		case 2:
			// triangle wave
			if(gradian<100){
				sPosition.setValue((float)((gradian+100)*(high-low)/200+low));
			}
			else if(gradian<300){
				sPosition.setValue((float)(gradian-100)*(low-high)/200+high);
			}
			else{
				sPosition.setValue((float)(gradian-300)*(high-low)/200+low);
			}
			break;
		case 3:
			// square wave
			if(gradian<100 || gradian>300){
				sPosition.setValue(high);
			}
			else{
				sPosition.setValue(low);
			}
			break;
		}
		// round position to nearest whole number
		sPosition.setValue(Math.round(sPosition.value()));
	}
	
	void communicate(){
		String inMessage;
		int inNumber = -1;
		
		//send set position
		myPort.write("SPOS"+ numberToString(sPosition.value())+'!');
		//update sent position on charts
		System.arraycopy(sentPositions,0,sentPositions,1,chartLength-1);
		sentPositions[0] = (float)sPosition.value();
		
		//ask for position, derivative and PD
		for(int i=0;i<3;i++){
			inNumber = -1;
			
			myPort.clear();
			switch(i){
			case 0:
				myPort.write("GPOS0000"+'!');
				break;
			case 1:
				myPort.write("GDRV0000"+'!');
				break;
			case 2:
				myPort.write("GPDV0000"+'!');
				break;
			}
			//wait 10 milliseconds and attempt to read
			delay(50);
			//read position
			if(myPort.available()<8){
				if(myPort.available()>0){
					println(myPort.readString());	
				}
				switch(i){
				case 0:
					chart1a.setText("none");
					break;
				case 1:
					chart2a.setText("none");
					break;
				case 2:
					chart3a.setText("none");
					break;
				}
			}
			else{
				inMessage = myPort.readString();
				if(inMessage.startsWith("SPOS", 0)){
					chart1a.setText(inMessage);
					inNumber = stringToNumber(inMessage.substring(4,8));
					//to indicate error, set value to -1
					if(inNumber>1023 || inNumber<0 ){
						inNumber = -1;
						println("position error: nonsence number");
					}
					//update received Position 
					System.arraycopy(receivePositions,0,receivePositions,1,chartLength-1);
					receivePositions[0] = (float)inNumber;
				}
				else if(inMessage.startsWith("SDRV")){
					chart2a.setText(inMessage);
					inNumber = stringToNumber(inMessage.substring(4,8));
					//update received derivative
					System.arraycopy(derivatives,0,derivatives,1,chartLength-1);
					derivatives[0] = (float)inNumber;
				}
				else if(inMessage.startsWith("SPDV")){
					chart3a.setText(inMessage);
					inNumber = stringToNumber(inMessage.substring(4,8));
					//update received PD
					System.arraycopy(PDvalues,0,PDvalues,1,chartLength-1);
					PDvalues[0] = (float)inNumber;
				}
			}
		}
	}
	
	void updateChart(){
		float[] tempArray = new float[chartLength];
		
		//scale data and draw them
		//sent positions
		for(int i=0;i<chartLength;i++){
			tempArray[i] = (sentPositions[i]-rRange.lowValue())
							/ (rRange.highValue()-rRange.lowValue())*100 + 14;
			if(tempArray[i]<0){
				tempArray[i] = 0;
			}
			if(tempArray[i]>128){
				tempArray[i] = 128;
			}
		}
		chart1.updateData(0, tempArray);
		
		//received position
		for(int i=0;i<chartLength;i++){
			tempArray[i] = (receivePositions[i]-rRange.lowValue())
							/ (rRange.highValue()-rRange.lowValue())*100 + 14;
			if(tempArray[i]<0){
				tempArray[i] = 0;
			}
			if(tempArray[i]>128){
				tempArray[i] = 128;
			}
		}
		chart1.updateData(1, tempArray);
		
		//derivative
		for(int i=0;i<chartLength;i++){
			tempArray[i] = derivatives[i]/(rRange.highValue()-rRange.lowValue())/2+64;
			if(tempArray[i]<0){
				tempArray[i] = 0;
			}
			if(tempArray[i]>128){
				tempArray[i] = 128;
			}
		}
		chart2.updateData(tempArray);
		
		//PDvalues
		for(int i=0;i<chartLength;i++){
			tempArray[i] = PDvalues[i]/2+64;
			if(tempArray[i]<0){
				tempArray[i] = 0;
			}
			if(tempArray[i]>128){
				tempArray[i] = 128;
			}
		}
		chart3.updateData(tempArray);
		
	}
	
	void updateRanges(){
		// round of ranges
		rRange.setLowValue((Math.round(rRange.lowValue())));
		rRange.setHighValue(Math.round(rRange.highValue()));
		
		// limit to Amplitude
		rAmplitude.setMin(rRange.lowValue());
		rAmplitude.setMax(rRange.highValue());
		
		if(rAmplitude.lowValue() < rAmplitude.min()){
			rAmplitude.setLowValue(rAmplitude.min());
		}
		if(rAmplitude.highValue() < rAmplitude.min()){
			rAmplitude.setHighValue(rAmplitude.min());
		}
		
		if(rAmplitude.highValue() > rAmplitude.max()){
			rAmplitude.setHighValue(rAmplitude.max());
		}
		if(rAmplitude.lowValue() > rAmplitude.max()){
			rAmplitude.setLowValue(rAmplitude.max());
		}
		
		// round amplitudes
		rAmplitude.setLowValue(Math.round(rAmplitude.lowValue()));
		rAmplitude.setHighValue(Math.round(rAmplitude.highValue()));
		
		// limits to position 
		sPosition.setMin(rRange.lowValue());
		sPosition.setMax(rRange.highValue());
		
		if(sPosition.value() < sPosition.min()){
			sPosition.setValue(sPosition.min());
		}
		if(sPosition.value() < sPosition.min()){
			sPosition.setValue(sPosition.min());
		}
		
		// round positions
		sPosition.setValue(Math.round(sPosition.value()));
	}
	
	public void setConstants() {
		Textfield input = null;

		for (int i = 0; i < 4; i++) {
			//loop through each text field input
			switch (i) {
			case 0:
				input = tSp;
				break;
			case 1:
				input = tAc;
				break;
			case 2:
				input = tKp;
				break;
			case 3:
				input = tKd;
				break;
			}
			try {
				//try to get usable data from text field 
				int value = Integer.parseInt(input.getText());
				if (value < 0 || value > 255) {
					throw new Exception("number out of range");
				}
				
				//format message to send
				String number = Integer.toString((int)((value%10000)/1000))
				+ Integer.toString((int)((value%1000)/100))
				+ Integer.toString((int)((value%100)/10))
				+ Integer.toString((int)((value%10)/1));
				
				switch (i) {
				case 0:
					myPort.write("SSPD" + number+'!');
					break;
				case 1:
					myPort.write("SACC" + number+'!');
					break;
				case 2:
					myPort.write("SCKP" + number+'!');
					break;
				case 3:
					myPort.write("SCKD" + number+'!');
					break;
				}
				
			} catch (Exception e) {
				input.setText("invalid");
			}
		}
	}
	
	//attempt to make a 4 digit String to an integer 
	int stringToNumber(String string){
		int number = ((int)(string.charAt(0)-'0'))*1000
					+((int)(string.charAt(1)-'0'))*100
					+((int)(string.charAt(2)-'0'))*10
					+((int)(string.charAt(3)-'0'));
		
		if(number>1023){
			number -=1023;
			number = -1* number;
		}
		
		return number;
	}
	
	//attempt to make a number into a 4 digit String
	String numberToString(float number){
		String string = Integer.toString((int)((number%10000)/1000))
					+ Integer.toString((int)((number%1000)/100))
					+ Integer.toString((int)((number%100)/10))
					+ Integer.toString((int)((number%10)/1));
		
		return string;
	}
	
	
//}

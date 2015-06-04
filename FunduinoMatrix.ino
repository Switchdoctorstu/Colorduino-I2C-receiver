/* Colorduino I2C handler
 *	This code is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *	
 *	This library is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *	Lesser General Public License for more details.

Understands Type 1 = char
			Type 2 = raster
			Type 3 = flip buffer
			Type 4 = Clear Screen

			
fixed packet lengths at 16 bytes

sorted timing issues... Careful with receive packet
Stuart Oldfield 2015

Thank you to too many web sites and sources from which i cut and copied and hacked
this together.

Working on:
Optimisation 
Serial Bus Chaining for auto module identification

*/




#include <Colorduino.h>
#include <Wire.h>
#define MODULEADDRESS 7 
#define BUFFERLENGTH 32 // 2 packets?
#define SERIALBUFFERLENGTH 64
#define CHARDISPLAY 1
#define BMPDISPLAY 2 

#define DEBUGI2C true
#define DEBUGDISPLAY true

typedef struct {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} ColorRGB;

typedef struct {
  unsigned char h;
  unsigned char s;
  unsigned char v;
} ColorHSV;
unsigned char plasma[ColorduinoScreenWidth][ColorduinoScreenHeight];
long paletteShift;
const unsigned char font_5x7[][5] = { // font array 
        { 0x00, 0x00, 0x00, 0x00, 0x00 },               /*   - 0x20 - 32 */
        { 0x00, 0x00, 0x5f, 0x00, 0x00 },               /* ! - 0x21 - 33 */ 
        { 0x00, 0x07, 0x00, 0x07, 0x00 },               /* " - 0x22 - 34 */ 
        { 0x14, 0x7f, 0x14, 0x7f, 0x14 },               /* # - 0x23 - 35 */
        { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },               /* $ - 0x24 - 36 */
        { 0x23, 0x13, 0x08, 0x64, 0x62 },               /* % - 0x25 - 37 */
        { 0x36, 0x49, 0x55, 0x22, 0x50 },               /* & - 0x26 - 38 */
        { 0x00, 0x05, 0x03, 0x00, 0x00 },               /* ' - 0x27 - 39 */
        { 0x00, 0x1c, 0x22, 0x41, 0x00 },               /* ( - 0x28 - 40 */
        { 0x00, 0x41, 0x22, 0x1c, 0x00 },               /* ) - 0x29 - 41 */
        { 0x14, 0x08, 0x3e, 0x08, 0x14 },               /* * - 0x2a - 42 */
        { 0x08, 0x08, 0x3e, 0x08, 0x08 },               /* + - 0x2b - 43 */
        { 0x00, 0x50, 0x30, 0x00, 0x00 },               /* , - 0x2c - 44 */
        { 0x08, 0x08, 0x08, 0x08, 0x08 },               /* - - 0x2d - 45 */
        { 0x00, 0x60, 0x60, 0x00, 0x00 },               /* . - 0x2e - 46 */
        { 0x20, 0x10, 0x08, 0x04, 0x02 },               /* / - 0x2f - 47 */
        { 0x3e, 0x51, 0x49, 0x45, 0x3e },               /* 0 - 0x30 - 48 */
        { 0x00, 0x42, 0x7f, 0x40, 0x00 },               /* 1 - 0x31 - 49 */
        { 0x42, 0x61, 0x51, 0x49, 0x46 },               /* 2 - 0x32 - 50 */
        { 0x21, 0x41, 0x45, 0x4b, 0x31 },               /* 3 - 0x33 - 51 */
        { 0x18, 0x14, 0x12, 0x7f, 0x10 },               /* 4 - 0x34 - 52 */
        { 0x27, 0x45, 0x45, 0x45, 0x39 },               /* 5 - 0x35 - 53 */
        { 0x3c, 0x4a, 0x49, 0x49, 0x30 },               /* 6 - 0x36 - 54 */
        { 0x01, 0x71, 0x09, 0x05, 0x03 },               /* 7 - 0x37 - 55 */
        { 0x36, 0x49, 0x49, 0x49, 0x36 },               /* 8 - 0x38 - 56 */
        { 0x06, 0x49, 0x49, 0x29, 0x1e },               /* 9 - 0x39 - 57 */
        { 0x00, 0x36, 0x36, 0x00, 0x00 },               /* : - 0x3a - 58 */
        { 0x00, 0x56, 0x36, 0x00, 0x00 },               /* ; - 0x3b - 59 */
        { 0x08, 0x14, 0x22, 0x41, 0x00 },               /* < - 0x3c - 60 */
        { 0x14, 0x14, 0x14, 0x14, 0x14 },               /* = - 0x3d - 61 */
        { 0x00, 0x41, 0x22, 0x14, 0x08 },               /* > - 0x3e - 62 */
        { 0x02, 0x01, 0x51, 0x09, 0x06 },               /* ? - 0x3f - 63 */
        { 0x32, 0x49, 0x79, 0x41, 0x3e },               /* @ - 0x40 - 64 */
        { 0x7e, 0x11, 0x11, 0x11, 0x7e },               /* A - 0x41 - 65 */
        { 0x7f, 0x49, 0x49, 0x49, 0x36 },               /* B - 0x42 - 66 */
        { 0x3e, 0x41, 0x41, 0x41, 0x22 },               /* C - 0x43 - 67 */
        { 0x7f, 0x41, 0x41, 0x22, 0x1c },               /* D - 0x44 - 68 */
        { 0x7f, 0x49, 0x49, 0x49, 0x41 },               /* E - 0x45 - 69 */
        { 0x7f, 0x09, 0x09, 0x09, 0x01 },               /* F - 0x46 - 70 */
        { 0x3e, 0x41, 0x49, 0x49, 0x7a },               /* G - 0x47 - 71 */
        { 0x7f, 0x08, 0x08, 0x08, 0x7f },               /* H - 0x48 - 72 */
        { 0x00, 0x41, 0x7f, 0x41, 0x00 },               /* I - 0x49 - 73 */
        { 0x20, 0x40, 0x41, 0x3f, 0x01 },               /* J - 0x4a - 74 */
        { 0x7f, 0x08, 0x14, 0x22, 0x41 },               /* K - 0x4b - 75 */
        { 0x7f, 0x40, 0x40, 0x40, 0x40 },               /* L - 0x4c - 76 */
        { 0x7f, 0x02, 0x0c, 0x02, 0x7f },               /* M - 0x4d - 77 */
        { 0x7f, 0x04, 0x08, 0x10, 0x7f },               /* N - 0x4e - 78 */
        { 0x3e, 0x41, 0x41, 0x41, 0x3e },               /* O - 0x4f - 79 */
        { 0x7f, 0x09, 0x09, 0x09, 0x06 },               /* P - 0x50 - 80 */
        { 0x3e, 0x41, 0x51, 0x21, 0x5e },               /* Q - 0x51 - 81 */
        { 0x7f, 0x09, 0x19, 0x29, 0x46 },               /* R - 0x52 - 82 */
        { 0x46, 0x49, 0x49, 0x49, 0x31 },               /* S - 0x53 - 83 */
        { 0x01, 0x01, 0x7f, 0x01, 0x01 },               /* T - 0x54 - 84 */
        { 0x3f, 0x40, 0x40, 0x40, 0x3f },               /* U - 0x55 - 85 */
        { 0x1f, 0x20, 0x40, 0x20, 0x1f },               /* V - 0x56 - 86 */
        { 0x3f, 0x40, 0x38, 0x40, 0x3f },               /* W - 0x57 - 87 */
        { 0x63, 0x14, 0x08, 0x14, 0x63 },               /* X - 0x58 - 88 */
        { 0x07, 0x08, 0x70, 0x08, 0x07 },               /* Y - 0x59 - 89 */
        { 0x61, 0x51, 0x49, 0x45, 0x43 },               /* Z - 0x5a - 90 */
        { 0x00, 0x7f, 0x41, 0x41, 0x00 },               /* [ - 0x5b - 91 */
        { 0x02, 0x04, 0x08, 0x10, 0x20 },               /* \ - 0x5c - 92 */
        { 0x00, 0x41, 0x41, 0x7f, 0x00 },               /* ] - 0x5d - 93 */
        { 0x04, 0x02, 0x01, 0x02, 0x04 },               /* ^ - 0x5e - 94 */
        { 0x40, 0x40, 0x40, 0x40, 0x40 },               /* _ - 0x5f - 95 */
        { 0x00, 0x01, 0x02, 0x04, 0x00 },               /* ` - 0x60 - 96 */
        { 0x20, 0x54, 0x54, 0x54, 0x78 },               /* a - 0x61 - 97 */
        { 0x7f, 0x48, 0x44, 0x44, 0x38 },               /* b - 0x62 - 98 */
        { 0x38, 0x44, 0x44, 0x44, 0x20 },               /* c - 0x63 - 99 */
        { 0x38, 0x44, 0x44, 0x48, 0x7f },               /* d - 0x64 - 100 */
        { 0x38, 0x54, 0x54, 0x54, 0x18 },               /* e - 0x65 - 101 */
        { 0x08, 0x7e, 0x09, 0x01, 0x02 },               /* f - 0x66 - 102 */
        { 0x38, 0x44, 0x44, 0x54, 0x34 },               /* g - 0x67 - 103 */
        { 0x7f, 0x08, 0x04, 0x04, 0x78 },               /* h - 0x68 - 104 */
        { 0x00, 0x44, 0x7d, 0x40, 0x00 },               /* i - 0x69 - 105 */
        { 0x20, 0x40, 0x44, 0x3d, 0x00 },               /* j - 0x6a - 106 */
        { 0x7f, 0x10, 0x28, 0x44, 0x00 },               /* k - 0x6b - 107 */
        { 0x00, 0x41, 0x7f, 0x40, 0x00 },               /* l - 0x6c - 108 */
        { 0x7c, 0x04, 0x18, 0x04, 0x78 },               /* m - 0x6d - 109 */
        { 0x7c, 0x08, 0x04, 0x04, 0x78 },               /* n - 0x6e - 110 */
        { 0x38, 0x44, 0x44, 0x44, 0x38 },               /* o - 0x6f - 111 */
        { 0x7c, 0x14, 0x14, 0x14, 0x08 },               /* p - 0x70 - 112 */
        { 0x08, 0x14, 0x14, 0x18, 0x7c },               /* q - 0x71 - 113 */
        { 0x7c, 0x08, 0x04, 0x04, 0x08 },               /* r - 0x72 - 114 */
        { 0x48, 0x54, 0x54, 0x54, 0x20 },               /* s - 0x73 - 115 */
        { 0x04, 0x3f, 0x44, 0x40, 0x20 },               /* t - 0x74 - 116 */
        { 0x3c, 0x40, 0x40, 0x20, 0x7c },               /* u - 0x75 - 117 */
        { 0x1c, 0x20, 0x40, 0x20, 0x1c },               /* v - 0x76 - 118 */
        { 0x3c, 0x40, 0x30, 0x40, 0x3c },               /* w - 0x77 - 119 */
        { 0x44, 0x28, 0x10, 0x28, 0x44 },               /* x - 0x78 - 120 */
        { 0x0c, 0x50, 0x50, 0x50, 0x3c },               /* y - 0x79 - 121 */
        { 0x44, 0x64, 0x54, 0x4c, 0x44 },               /* z - 0x7a - 122 */
        { 0x00, 0x08, 0x36, 0x41, 0x00 },               /* { - 0x7b - 123 */
        { 0x00, 0x00, 0x7f, 0x00, 0x00 },               /* | - 0x7c - 124 */
        { 0x00, 0x41, 0x36, 0x08, 0x00 },               /* } - 0x7d - 125 */
        { 0x10, 0x08, 0x08, 0x10, 0x08 },               /* ~ - 0x7e - 126 */
};

// setup receive buffers 
char receivebuffer[BUFFERLENGTH];
char wirebuffer[BUFFERLENGTH];
char serialbuffer[SERIALBUFFERLENGTH];
boolean serialpacketavailable=false;
int serialcursor=0;
int buffercursor=0;
int oldbuffercursor=0;
int packets=0;
boolean packetAvailable=false;
int displaymode=0;
int oldchar;
int moduleID=MODULEADDRESS;

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
	while(0 < Wire.available()) // loop through all
	{
		char c = Wire.read(); // receive byte as a character
		
		if(buffercursor<BUFFERLENGTH){
			wirebuffer[buffercursor]=c; // put it in the buffer
			buffercursor++;
			if ((buffercursor>=16)&&(c==0x0d)){
				packetAvailable=true;
				if(DEBUGI2C){
					Serial.println("M:"+ String(moduleID)+" Len"+String(buffercursor));
				}		
			}
		} 
		else{
			if(DEBUGI2C) Serial.println("I2C Buffer Overrun");
			buffercursor=0; // reset the buffer
		}
		
	}
	
	packets++;
       
 }

void setup() {
  
  Colorduino.Init();
  // compensate for relative intensity differences in R/G/B brightness
  // array of 6-bit base values for RGB (0~63)
  // whiteBalVal[0]=red
  // whiteBalVal[1]=green
  // whiteBalVal[2]=blue
  unsigned char whiteBalVal[3] = {36,63,63}; // for LEDSEE 6x6cm round matrix
  Colorduino.SetWhiteBal(whiteBalVal);
// Start Listening on i2C bus
  Wire.begin(MODULEADDRESS);                // join i2c bus with address set at top
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(19200);           // start serial for output
	if(DEBUGDISPLAY)Serial.println("DEBUGDISPLAY");
	if(DEBUGI2C)Serial.println("DEBUGI2C");
	
}

void loop(){
  if(packetAvailable) packet();
  if(Serial.available()>0)
			{
			GetSerialData();
		}
	if(serialpacketavailable){
		checkmessage();
	}
}

void checkmessage(){  

// See if the message is for us
	if(serialbuffer[0]==moduleID){
		// say hi
		Serial.println(String(moduleID));
	}
	serialcursor=0;
	serialpacketavailable=false;
}

void GetSerialData(){
	char lf = 0x0A;
	char c =Serial.read();
	serialbuffer[serialcursor]=c;
	if(c==lf){
		serialpacketavailable=true;
	}	
	if(serialcursor<SERIALBUFFERLENGTH){
		serialcursor++;
	}
	else{
		serialcursor=0; // wrap the buffer 
	}
}

void packet(){
	// we have some data
	// copy from wirebuffer to receive buffer
	int length=0;
	int dc;
	char red,blue,green;
	char c;
	char display_char;
	int type;
	unsigned int x,y;
	unsigned char mybyte;
	float value;
	ColorRGB colorRGB;
	ColorHSV colorHSV;
	int fontheight=7;
	int fontwidth=5;
	int tx;
	PixelRGB *p;
	
	do{  // grab a copy of the buffer before we get another packet
		c=wirebuffer[length];
		//if(DEBUGI2C)	{
		//	Serial.print(c,HEX);
		//	Serial.print(" ,");
		//}
		receivebuffer[length]=c;
		length++;
	} while (length<BUFFERLENGTH);
	buffercursor=0; // reset the buffer
	packetAvailable=false;
	type=receivebuffer[1];
	if(DEBUGI2C)	Serial.println(" Type:"+String(type)+" Packet");
	/*
	Type 	1 	char	   		character to display
			2	byte[64]		red bitmap
			3	byte[64]		red bitmap
			4	byte[64]		red bitmap
			5	byte x,y,r,g,b 	set x,y to r,g,b
	*/
	switch(type){
		case 1: {
			//  0	'#' 
			// 	1	type = 0x01
			//  2  	character in ASCII
			// 	3	red
			//	4	blue
			// 	5	green
			// ... padding to make whole packet 16 bytes
			// etx = 0x0d
			display_char=receivebuffer[2];
			if(DEBUGI2C){
				Serial.print("M:"+ String(moduleID)+" Character Received:");
				Serial.println(display_char,HEX);
			}
			dc=display_char-32;
			
			if(dc!=oldchar){
				oldchar=dc;
				// clear the screen
				ColorFill(0,0,0);
			}		
			red=receivebuffer[3];
			blue=receivebuffer[4];
			green=receivebuffer[5];
			// draw the character
			for(y = 0; y < fontheight; y++){
				for(x = 0; x < fontwidth; x++) {
					
					// check to see if bit should be set
					// get bit mask
					mybyte = font_5x7[dc][x];
					mybyte=mybyte&(64>>y);
					tx=x+1;
					if(mybyte>0){ 
						Colorduino.SetPixel(tx, y, red, blue, green);
						if(DEBUGDISPLAY)Serial.print("1");
					}
					else{
						Colorduino.SetPixel(tx,y,0,0,0);
						if(DEBUGDISPLAY)Serial.print("0");
					}
				}	
				if(DEBUGDISPLAY)Serial.println();
			}
				
			Colorduino.FlipPage(); // swap screen buffers to show it
			break;
		}
		
		case 2:  {// bitmap
			// we've got a raster so lets set some bits
			// protocol goes:
			//  0 	stx
			//  1 	0x02   = type 2 = raster
			//  2 	0xLC = Line(0-7) Colour(0-2) 0=red 1=blue 2=green
			// 	3-10 	byte x 8    intensity data for that line
			// 	11	padding to 16 bytes
			// 	15 	etx = 0x0d
			
			// get colour
			int colour=receivebuffer[2] & 0x0F;
			// get row
			unsigned int row = (receivebuffer[2] &0xF0) /16;
			// add row to pointer
			p = Colorduino.GetPixel(0,row);
			// set pixel row
			if(DEBUGI2C){	
				Serial.print(" R:"+String(row)+" C:");
				Serial.println(colour,HEX);
			}
			for ( y=0;y<8;y++) {  // should be width of screen
				p = Colorduino.GetPixel(y,row);
				unsigned char value=receivebuffer[y+3];  // get pixel value
				if(colour==0) 	p->r = value;	
				if(colour==1) 	p->g = value;	
				if(colour==2)	p->b = value;	
				// Colorduino.SetPixel(row,y,r,g,b)
				if(DEBUGDISPLAY)Serial.print(value,HEX);
			}	
			if(DEBUGDISPLAY)Serial.println();
			//Colorduino.FlipPage();
		
			break;
		}
		case 3:{
			Colorduino.FlipPage();
			break;
		}
		case 4:{
			// clear screen to 
			// protocol goes:
			//  0 	stx
			//  1 	0x04   = type 4 = Fill
			//  2 	red
			// 	3	green
			//	4  	blue
			red=receivebuffer[2];
			blue=receivebuffer[3];
			green=receivebuffer[4];
			ColorFill(red,blue,green);
		}
	}
}

/********************************************************
Name: ColorFill
Function: Fill the frame with a color
Parameter:R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
********************************************************/
void ColorFill(unsigned char R,unsigned char G,unsigned char B)
{
  PixelRGB *p = Colorduino.GetPixel(0,0);
  for (unsigned char y=0;y<ColorduinoScreenWidth;y++) {
    for(unsigned char x=0;x<ColorduinoScreenHeight;x++) {
      p->r = R;
      p->g = G;
      p->b = B;
      p++;
    }
  }
  
   Colorduino.FlipPage();
}

void HSVtoRGB(void *vRGB, void *vHSV) {//Converts an HSV color to RGB color
  float r, g, b, h, s, v; //this function works with floats between 0 and 1
  float f, p, q, t;
  int i;
  ColorRGB *colorRGB=(ColorRGB *)vRGB;
  ColorHSV *colorHSV=(ColorHSV *)vHSV;

  h = (float)(colorHSV->h / 256.0);
  s = (float)(colorHSV->s / 256.0);
  v = (float)(colorHSV->v / 256.0);

  //if saturation is 0, the color is a shade of grey
  if(s == 0.0) {
    b = v;
    g = b;
    r = g;
  }
  //if saturation > 0, more complex calculations are needed
  else
  {
    h *= 6.0; //to bring hue to a number between 0 and 6, better for the calculations
    i = (int)(floor(h)); //e.g. 2.7 becomes 2 and 3.01 becomes 3 or 4.9999 becomes 4
    f = h - i;//the fractional part of h

    p = (float)(v * (1.0 - s));
    q = (float)(v * (1.0 - (s * f)));
    t = (float)(v * (1.0 - (s * (1.0 - f))));

    switch(i)
    {
      case 0: r=v; g=t; b=p; break;
      case 1: r=q; g=v; b=p; break;
      case 2: r=p; g=v; b=t; break;
      case 3: r=p; g=q; b=v; break;
      case 4: r=t; g=p; b=v; break;
      case 5: r=v; g=p; b=q; break;
      default: r = g = b = 0; break;
    }
  }
  colorRGB->r = (int)(r * 255.0);
  colorRGB->g = (int)(g * 255.0);
  colorRGB->b = (int)(b * 255.0);
}

float dist(float a, float b, float c, float d) 
{
  return sqrt((c-a)*(c-a)+(d-b)*(d-b));
}


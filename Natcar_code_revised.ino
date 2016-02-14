//////////////////////////////////////////////////////////////////////
//
//  Natcar image processing code utilizing TSL1401 line scan camera
//
//  Purpose: To facilitate the use of the TSL1401 camera
//           easily for line following applications utilizing
//           information found in the datasheet
//
//  Author:  Ian Mackie
//
//  Date:    12/13/15
//
//////////////////////////////////////////////////////////////////////

byte CLKpin = 12;              //Define clock pin as pin 12
byte SIpin = 11;               //Define shiftout pin as pin 11
int A0pin =  A0;               //Define analog output pin as pin 10

//////////////////////////////////////////////////////////////////////
//Global variables
byte pixelBuffer[128];           //Buffer to hold pixel information
String line;

//////////////////////////////////////////////////////////////////////

void setup()
{
 Serial.begin(9600);           //Begin serial connection for debugging
 pinMode(CLKpin, OUTPUT);      //
 pinMode(SIpin, OUTPUT);       //Define utilized pins as input/output
 pinMode(A0pin, INPUT);  
}

void loop()
{
  getCamera();                        //Call camera function
  printPixels();                      //Print pixel array, starting a new line with each picture
  Serial.println('\n');               //Start each picture on a new line
}

void getCamera()                //function to expose pixels and read them back for processing
{
  digitalWrite(SIpin, HIGH);    //Pull pin high to signal begin capture sequence
  for(byte i = 0; i < 128; i++)
  {
    digitalWrite(CLKpin, HIGH);   //Pull pin high to signal begin of capture sequence
    digitalWrite(SIpin, LOW);     //Pull pin low again to actually begin collecting data
    pixelBuffer[i] = analogRead(A0pin);         //Read in pixel info with each pulse starting with MSB(left side)
    digitalWrite(CLKpin, LOW);
  }
}

void printPixels()                        //test function to verify code is working
{
  for(byte i = 0; i < 128; i++)
  {
     Serial.println(pixelBuffer[i]);      //Print each element of the camera array
  }
}

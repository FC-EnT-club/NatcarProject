//////////////////////////////////////////////////////////////////////
//
//  Natcar image processing code utilizing TSL1401 line scan camera
//
//  Purpose: To facilitate the use of the TSL1401 camera
//           to determine the position of a line and use
//           that information to follow it by steering using
//           a servo motor.
//
//  Author:  Ian Mackie
//
//  Date:    02/26/16
//
//////////////////////////////////////////////////////////////////////

//Note: Please ignore the commented out code, it will be used for the C++ version of this program
//Note: In the final version of the program all debugging lines will be removed
//Note: Variables that get updated, are done so as global, instead of by reference 

#include <Arduino.h>
#include <Servo.h>

//////////////////////////////////////////////////////////////////////
//Definition Macros, for static variables that do not change
#define uBound 1770            //Change this to edit upper servo bounds       <=This changes depending on frame and servo
#define lBound 1125            //Change this to edit lower servo bounds       <=Same as above
#define camCenter 64           //Camera center pixel value                    <=Test this for accuracy
#define cntrlPin1 A0           //Pin reserved for controller input/output
#define cntrlPin2 A1           //Pin reserved for controller input/outout
#define cntrlPin3 A2           //Pin reserved for controller input/output
#define servoPin 2             //Pin reserved for controlling steering servo 

//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//Pin Definitions                                                             <=Might consider using definition macros here as well
byte CLKpin = 12;               //Define clock pin as pin 12
byte SIpin = 11;                //Define shiftout pin as pin 11
int A0pin =  10;                //Define digital output pin as pin 10

//////////////////////////////////////////////////////////////////////
//Global variables
volatile byte mode = 0;          //Mode flag variable, 1 for manual & 0 for automatic
byte pixelBuffer[128];           //Buffer to hold pixel information
byte zCnt = 0;                   //Variable to count zero pixels before line detection
byte error = 0;                  //Error variable
byte prevEr = 0;                 //Previous error variable
float kP = 0;                    //Proprtion tuning variable
float kI = 0;                    //Integration tuning variable
float kD = 0;                    //Derivative tuning variable
int Perror = 0;                  //Proportion error variable
int Ierror = 0;                  //Integreation error variable
int Derror = 0;                  //Derivative error variable
int heading = 0;                 //Variable to hold steering information
int timeLapse = 0;               //Variable to hold rate of change variable (Final time - Initial time)
int iTime = 0;                   //Variable to hold initial time                                                   <=We need to see if its better to measure rate of change, or simple aggregated values
int fTime = 0;                   //Variable to hold final time
byte tempBuff = 0;               //Temp buffer to hold aggregated picture data (for debugging)
Servo dControl;                  //Generate Servo class object

//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//Setup Function
void setup()
{
 Serial.begin(9600);                         //Begin serial connection (For debugging)
 dControl.attach(servoPin);                  //Attach servo control pin to pin
 //pinMode(CLKpin, OUTPUT);      
 //pinMode(SIpin, OUTPUT);       
 //pinMode(A0pin, INPUT);
 DDRB = 0b00011000;                          //CLKpin, SIpin outputs, A0pin and rest inputs
 DDRC = 0b00000111;                          //Pin A0 - A2 inputs, rest outputs
 PORTB = 0b00000000;                         //Pull all pins low
 PORTC = 0b00000111;                         //Enable internal pull-up resistors pins A0 - A2
 //Enable interrupts for manual/auto modes
 //Start phase-correct PWM hardware
 //Start comms if need be  
}
///////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
//Main Loop
void loop()
{
  zCnt = 0;                                   //Reset zero counter for each picture
  iTime = millis();                           //Acquire beginning time of program
  PORTB |= (1 << SIpin);                      //Pull pin high to ready data for transfer
  //digitalWrite(SIpin, HIGH);                //Pull pin high to signal begin capture sequence
  for(byte i = 0; i < 128; i++)
  {
    PORTB |= (1 << CLKpin);                   //Pull clock pin high to signal begin of capture sequence
    //digitalWrite(CLKpin, HIGH); 
    //digitalWrite(SIpin, LOW);     
    PORTB &= ~(1 << SIpin);                   //Pull pin low again to actually begin collecting data
    //pixelBuffer[i] = digitalRead(A0pin);       
    pixelBuffer[i] = (PINB & (1 << A0pin));   //Read in pixel info (might need to change AOpin to 2)
    PORTB &= ~(1 << CLKpin);                  //Toggle clock pin to cycle in next pixel
    //digitalWrite(CLKpin, LOW);
  }
  countPixels();                              //Count zeros until a 1 is found, indicating the line
  fTime = millis();                           //Acquire ending time of program
  timeLapse = fTime - iTime;                  //Calculate time rate of change
  pidCalc();                                  //Calculate PID information for steering
  boundCheck();                               //Check to make sure servo 
  dControl.writeMicroseconds(heading);        //Actually move servo with respect to calculated direction
  //Send motor info to other arduino micro if need be
  Serial.println('\n');                       //Start each picture on a new line (For debugging)
  delay(3000);                                //Delay for visually confirming values (For debugging)
}
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////
//Function to bound servo value within spec
void boundCheck()
{
  if(heading > uBound || heading < lBound)    //Check to see if servo value is out of spec
  {
    if(heading > uBound)                      //Value too high
    {
      heading = uBound;                       //Bound to highest value
    }
    else                                      //Value too low
    {
      heading = lBound;                       //Bound to lowest value
    }
  }
}
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
//Zero Pixel Function
void countPixels()                            //Counts pixels until line is detected
{
  for(byte i = 0; i < 128; i++)               //Count through buffer array holding picture Note: if we count from center line, we count less
  {
    if(pixelBuffer[i] == 0)                   //Count the number of zeros
    {
      zCnt += 1;                              //Increment zero counter to generate pError val
    }
    tempBuff += (pixelBuffer[i]);             //Aggregate each element of the camera array (For debugging)
  }
  Serial.println(tempBuff);                   //Print compilied picture information (For debugging)
}
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
//PID Function
void pidCalc()                                             //Calculate component values used for PID algorithm (Note: values are updated each main loop iteration)
{
  error = camCenter - zCnt;                                //Generate error values based on counted zeros
  Ierror = (Ierror + error) * timeLapse;                   //Generate integration error value
  Derror = (error - prevEr) / timeLapse;                   //Generate derivative error value
  prevEr = error;                                          //Update previous error value
  heading = (kP * error) + (kI * Ierror) + (kD * Derror);  //Generate directional information from user set gains and error information
}
////////////////////////////////////////////////////////////////////////


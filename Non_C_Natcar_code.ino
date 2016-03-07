////////////////////////////////////////////////////////////////////////////////
//
//  Program: C++ version of the 2016 Natcar competition code
//
//  Purpose: To teach the functionality of the logic and implementation
//           of the ideal and comcepts utilized in the Natcar code
//           through a medium that is easier to understand
//
//  Author:  Ian Mackie
//
//  Date: 3/06/2016
//
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//Included libraries required for the program to work properly
#include <Arduino.h>
#include <Servo.h>
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//Pin declarations
byte CLKpin = 12;             //Define clock pin as pin 12
byte SIpin = 11;              //Define shiftout pin as pin 11
int A0pin =  10;              //Define digital output pin as pin 10
byte cntrlPin1 = 9;           //Pin reserved for controller input/output
byte cntrlPin2 = 8;           //Pin reserved for controller input/outout
byte cntrlPin3 = 7;           //Pin reserved for controller input/output
byte servoPin = 2;            //Pin reserved for controlling steering servo
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
//Global Variables
volatile byte mode = 0;          //Mode flag variable, 1 for manual & 0 for automatic
int uBound = 1770;            //Change this to edit upper servo bounds       <=This changes depending on frame and servo
int lBound = 1125;            //Change this to edit lower servo bounds       <=Same as above
byte camCenter = 64;          //Camera center pixel value                    <=Test this for accuracy
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
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//Setup loop to 
void setup() 
{
  pinMode(A0pin, INPUT);                      //Define A0pin as an input
  pinMode(SIpin, OUTPUT);                     //Define SIpin as an output
  pinMode(CLKpin, OUTPUT);                    //Define CLKpin as an output
  dControl.attach(servoPin);                  //Attach servo control pin to servo pin
  Serial.begin(9600);                         //Begin serial connection (For debugging)
}
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//Main loop where the main part of the program executes
void loop() 
{
  zCnt = 0;                                   //Reset zero counter for each picture
  iTime = millis();                           //Acquire beginning time of program
  digitalWrite(SIpin, HIGH);                  //Pull SIpin high to begin picture exposure
  for(byte i = 0; i < 128; i++)               //For loop to shift through each element of 128 pixel picture
  {
    digitalWrite(CLKpin, HIGH);               //Pull clock pin high to signal begin of capture sequence
    digitalWrite(SIpin, LOW);                 //Pull SIpin low again to actually begin collecting data     
    pixelBuffer[i] = digitalRead(A0pin);      //Read in picture information from camera
    digitalWrite(CLKpin, LOW);                //Toggle clock pin to pull in next pixel
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
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//Function to bound servo value within spec to prevent damage to servo
void boundCheck()
{
  if(heading > uBound || heading < lBound)    //Check to see if servo value is above or below spec
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
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////
//PID Function
void pidCalc()                                             //Calculate component values used for PID algorithm (Note: values are updated each main loop iteration)
{
  error = camCenter - zCnt;                                //Generate error values based on counted zeros
  Ierror = (Ierror + error) * timeLapse;                   //Generate integration error value
  Derror = (error - prevEr) / timeLapse;                   //Generate derivative error value
  prevEr = error;                                          //Update previous error value
  heading = (kP * error) + (kI * Ierror) + (kD * Derror);  //Generate directional information from user set gains and error information
}
///////////////////////////////////////////////////////////////////////////////////


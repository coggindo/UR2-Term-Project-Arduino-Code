// include libraries 
#include <Servo.h>            // include for mini and base servos
#include <SoftwareSerial.h>   // include for DC motor serial communication 

///////////////////////////////////////////////////////////////////////
// setup pins 
// DC motor pins -- DC motor moves 0.780 in/sec at 1200 speed
#define rxPin 3 // pin 3 connects to smcSerial TX (not used in this example)
#define txPin 4 // pin 4 connects to smcSerial RX
SoftwareSerial smcSerial = SoftwareSerial(rxPin, txPin);
double timeMotorMove;   

// servo pins and servo initialization
int miniServoPin = 11;
Servo miniServo;
int baseServoPin = 6; 
Servo baseServo;

// magnet pin
int magnetPin = 9;

// C# serial communication setup 
const int ledPin = 13; // the pin that the LED is attached to
const byte buffSize = 40;
unsigned int inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
byte coordinates[3];    // {xCoordinate, yCoordinate, shape}

double xPosition[2];    // {distanceFromCenterOfSheet, sideOfPaper}
double yPosition;       // distance from bottom of paper 

///////////////////////////////////////////////////////////////////////
// actual arduino code 
void setup() {
  
  // setup serial communication
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // set up DC motor controller
  // Initialize software serial object with baud rate that matches the other serial.
  smcSerial.begin(115200);

  // The Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms.
  delay(5);

  // If the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate.
  smcSerial.write(0xAA);

  // Next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run.
  exitSafeStart();
  
  // attach servos to their pins
  miniServo.attach(miniServoPin);
  baseServo.attach(baseServoPin);

  // activate magnet pin as an output pin
  pinMode(magnetPin, OUTPUT);

  // initialize servo angles and DC motor speed
  miniServo.write(50);
  baseServo.write(95);
  setMotorSpeed(0);

}

void loop() 
{
  // get information from c# code 
  getDataFromPC();
  if(newDataFromPC)
  {
    // light up pin 13 LED to confirm communication 
    digitalWrite(ledPin, HIGH);
    delay(2000);
    digitalWrite(ledPin, LOW);

    // set the x and y positions in inches based on the input 
    // from the c# code in pixels 
    determinePositionX(coordinates[0]);
    determinePositionY(coordinates[1]);

    // create local variables based on the position of the
    // shapes in inches
    double x[2] = {xPosition[0], xPosition[1]};   // {distanceFromCenterOfPaperInInches, sideOfPaper}
    double y = yPosition;                         // distanceFromBottomOfPaperInInches

    // call distanceFromBase(). This function does all calculations and also
    // calls each function to move the robot
    distanceFromBase(x[0], x[1], yPosition);

    // return the coordinates to the forms app 
    sendCoordinatesToPC();

    // set newData to false in order to interpret new information
    newDataFromPC = false;
  }
    // set all motor speeds to 0 to keep motor from moving
    setMotorSpeed(0);
}

///////////////////////////////////////////////////////////////////////
// calculation functions 

void determinePositionX(double x)
{
     // this function takes the coordinate in pixels and changes it into 
     // inches. It also determines which side of the paper the shape is on
     
     double centerXVal = 105; // (105,y) is center line
     double pixelOffset = x - centerXVal; // distance from center of the paper

     double paperSide = 0;  //determines if shape is on left or right side of paper

     // depending on the sign of pixelOffset, it sets paperSide = 1 or 2 
     // to represent the left or right side of the paper
     if (pixelOffset < 0)
     {
       // shape on left side of paper
       paperSide = 1;
     }
     else if (pixelOffset > 0)
     {
       // shape on right side of paper
       paperSide = 2;
     }

     // take the absolute value of pixelOffset 
     pixelOffset = abs(pixelOffset);
     
     double frameWidth = 210;   // frame width in pixels 
     double temp = pixelOffset/frameWidth;  // temporary variable
     double inchesFromCenter = temp * 11;   // takes temp and multiplies by width of paper 
     // width of paper is 11 in this case because paper is landscape 

     // set global variable to these values 
     xPosition[0] = inchesFromCenter;
     xPosition[1] = paperSide;
}

void determinePositionY(double y)
{
     // this function takes the coordinate in pixels and converts
     // it into inches from the bottom of the page
     
     double frameHeight = 228;    // frameHeight in pixels
     double temp = frameHeight - y; // temporary variable
     double inchesFromBottom = (temp / frameHeight) * 8.5;  //takes temp and multiplies by height of paper
     // height of paper is 8.5 because paper is landscape

     yPosition = inchesFromBottom;    // inches from bottom of page 
}

double findRotationAngle(double x, double y, int angleSign)
{
  // this function determines the rotation angle of the base servo
  // based on the distance from the center of the page (x), distance
  // from the servo to the shape (y), and which side (left or right)
  // of the paper the shape is on (angleSign)
  
  double angle = atan(x/y);
  angle = angle * 180 / PI;

  if (angleSign == 1)
  {
    // <= 5.5 inches from left of page
    angle = 100 + angle;
  }
  else if (angleSign == 2)
  {
    // > 5.5 in from left of page
    angle = 100 - angle; 
  }
 
  return angle;
}

void distanceFromBase(double inchesFromCenterX, double paperSideX, double inchesFromBottomY)
{
   if(coordinates[0] == 0 && coordinates[1] == 0)
   {
    // no shapes are found because there is no location where
    // the center of the shape is recognized at (0,0)
    return;
   }

   // determine distance from servo to shape 
   double distanceFromBaseToPaper = 7.75;    //edge of 
   double distanceFromServoToEdgeBase = 0;   // 1 + 13/16 inches 
   // i didn't alter the above variable because i didn't add it until test day 
   double distanceFromServoToShape = distanceFromBaseToPaper + distanceFromServoToEdgeBase + inchesFromBottomY;
   
   // call findRotationAngle function
   double rotationAngle = findRotationAngle(inchesFromCenterX, distanceFromServoToShape, paperSideX);
   
   // find hypotenuse using the pythagorean theorem
   double hypotenuse = sqrt((distanceFromServoToShape * distanceFromServoToShape) + (inchesFromCenterX * inchesFromCenterX));

   // rotate the base servo to the given angle
   baseServo.write(rotationAngle);
   delay(3000);
   
   // extend arm using moveDCMotor function
   moveDCMotor(hypotenuse, inchesFromBottomY);
   delay(3000);
}

///////////////////////////////////////////////////////////////////////
// base servo rotation functions
void rotateToShapePile()
{
  // rotate to pile based on shape 
  // 1 = triangle, 2 = square 
  if (coordinates[2] == 1)
  {
    // rotate to left side to the triangle pile
    baseServo.write(180);
    delay(600);
  }
  else if (coordinates[2] == 2)
  {
    // rotate to right side to the square pile 
    baseServo.write(0);
    delay(600);
  }

  // call the drop shape function
  dropShape();
}

///////////////////////////////////////////////////////////////////////
// mini servo rotation functions 
void pickUpShape()
{
  // this function writes to the mini servo and magnet
  // to pick up the shape 
  
  // drop magnet
  miniServo.write(0);
  delay(500);
  
  // activate magnet
  digitalWrite(magnetPin, HIGH);
  delay(4000);

  // raise magnet
  miniServo.write(80);
  delay(500);
}

void dropShape()
{
  // this function writes to the mini servo and magnet
  // to drop up the shape 
  
  // drop magnet
  delay(500); 
  miniServo.write(20);
  delay(500);

  //deactivate magnet
  digitalWrite(magnetPin, LOW);
  delay(1000);

  // raise magnet
  miniServo.write(50);
  delay(500);
}

///////////////////////////////////////////////////////////////////////
// dc motor functions
void moveDCMotor(double hypotenuse, double inFromBottom)
{
  // this function moves the DC motor based on the shape's position
  
  double minExtension = 7.75;   // minimum extension of robotic arm to match with edge of paper 
  double moveMotorQuantity = hypotenuse - minExtension; // how far the arm needs to extend 

  // at speed 1200, motor moves 0.780 inches / second 
  double inchPerSecond = 0.780;   // can be changed based on speed the arm moves at 
  timeMotorMove = moveMotorQuantity / inchPerSecond;
  
  timeMotorMove = timeMotorMove * 500;  // multiply by 500 to get to appropriate ms 
  double totalMoveForward = timeMotorMove + 300;  // add 300 ms 

  double addTime = 0; // create addTime variable to adjust timing based on how far the shape is from edge of paper
  if (inFromBottom >= 7.0)
  {
    // center of shape is >= 7.0" from edge of paper
    addTime = 800;
  }
  else if (inFromBottom < 7.0 && inFromBottom > 5.75)
  {
    // center of shape is > 5.5" and < 7" from edge of paper
    addTime = 500;
  }
  else if (inFromBottom <= 5.75 && inFromBottom > 4.25)
  {
    // center of shape is <= 5.5" and > 4.5" from paper
    addTime = 200;
  }
  else if (inFromBottom <= 4.25 && inFromBottom > 2.5)
  {
    // center of shape is <= 4.25" and > 2.5" from paper
    addTime = -150;
  }
  else if(inFromBottom <= 2.5)
  {
    // center of shape is <= 2.5" from edge of paper
    addTime = -550;
  }
  
  // move motor forward based on position
  setMotorSpeed(1200);  // set speed
  delay(timeMotorMove + addTime); //motor runs for the delay time
  
  // pause motor, activate magnet
  setMotorSpeed(0); //stop motor
  delay(1500);
  pickUpShape();    // pick up shape with function call
  delay(1500);

  // reverse motor   
  setMotorSpeed(-1200);   // set same speed in opposite direction
  delay(timeMotorMove + addTime); // motor runs for delay time
 
  // stop motor
  setMotorSpeed(0);
  delay(300);

  // move rack forward so magnet clears edge of base 
  setMotorSpeed(2300);
  delay(600);

  // stop motor
  setMotorSpeed(0);
  delay(600);
  
  // move to side based on shape and drop shape 
  rotateToShapePile();

  //return to 90 degrees + servo offset
  baseServo.write(95);
  delay(700);

  //move back to start position on rack 
  setMotorSpeed(-2300);
  delay(600);

  // stop
  setMotorSpeed(0);
  
}

// received function from Pololu Simple Motor Controller Manual 
// Sample Code for Arduino
void exitSafeStart()
{
  // required to allow motors to move
  // must be called when controller restarts and after any error
  smcSerial.write(0x83);
}

// received function from Pololu Simple Motor Controller Manual 
// Sample Code for Arduino
void setMotorSpeed(int speed)
{
  // speed should be a number from -3200 to 3200
  if (speed < 0)
  {
    smcSerial.write(0x86); // motor reverse command
    speed = -speed; // make speed positive
  }
  else
  {
    smcSerial.write(0x85); // motor forward command
  }
  smcSerial.write(speed & 0x1F);
  smcSerial.write(speed >> 5 & 0x7F);
}

///////////////////////////////////////////////////////////////////////
// serial c# communication functions
// received both of these functions from Lab 8 
void sendCoordinatesToPC()
{
  // send the point data to the PC
  Serial.print("<P");
  Serial.print(coordinates[0]);
  Serial.print(",");
  Serial.print(coordinates[1]);
  Serial.print(",");
  Serial.print(coordinates[2]);
  Serial.println(">");
}

void getDataFromPC() 
{
  // alternative to the readBytes function:
  // receive data from PC and save it into inputBuffer
  if(Serial.available() > 0) 
  {
    char x = Serial.read();
    // the order of these IF clauses is significant
    if (x == endMarker) 
    {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      coordinates[0] = inputBuffer[0];
      coordinates[1] = inputBuffer[1];
      coordinates[2] = inputBuffer[2];
    }
    if(readInProgress) 
    {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) 
      {
        bytesRecvd = buffSize - 1;
      }
    }
    if (x == startMarker) 
    {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

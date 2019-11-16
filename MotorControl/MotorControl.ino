#include <Arduino.h>;
#include <digitalWriteFast.h>

//Pin 2 is FrontBack INTerrupt from encoders
//Pin 3 is LeftRight Interrupt from encoders
//Pin 6 and 7 are front back control
//Pin 8 and 9 are Left right control

// Assign motors to pins
//#define top 6
//#define bottom 7
//#define left 8
//#define right 9

// Ticks since last measurement
volatile int topTicks = 0;
volatile int bottomTicks = 0;
volatile int leftTicks = 0;
volatile int rightTicks = 0;

// Robot dimensions (in inches)
const double yDimension = 16.75;
const double xDimension = 23.9375;
const double distanceFactor = 0.007955784;
const double motorFactor = 0.52466368;

double currentX = xDimension;
double currentY = yDimension;

void addTickTop() {
  topTicks++;
}

void addTickBottom() {
  bottomTicks++;
}

void addTickLeft() {
  leftTicks++;
}

void addTickRight() {
  rightTicks++;
}

// Interrupt pins: 2, 3, 18, 19, 20, 21
void setup() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(18), addTickTop, RISING);
  attachInterrupt(digitalPinToInterrupt(19), addTickBottom, RISING);
  attachInterrupt(digitalPinToInterrupt(20), addTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(21), addTickRight, RISING);
}

void loop() {

  topTicks = 0;
  bottomTicks = 0;
  leftTicks = 0;
  rightTicks = 0;
  currentX = getXDistance();
  currentY = getYDistance();
//  moveForwardBackwards(1024,100);
//  delay(1000);
//  moveLeftRight(1024,100);
//  delay(1000);
//  moveForwardBackwards(1024,-100);
//  delay(1000);
//  moveLeftRight(1024,-100);
//  delay(1000);
  
}

void moveForwardBackwards(int dist, int dir) { //Distance is in mm Direction is positve is forward negative is back. Abs value of direction is speed
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
   
  if (dir > 0) {
    analogWrite(6,abs(dir));
    digitalWrite(7,LOW);
     
  }
  else {
    analogWrite(7,abs(dir));
    digitalWrite(6,LOW);
    
  }
  
//  while(distanceFB <= dist/3.5);
//  distanceFB = 0;
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
}

double getXDistance() {
  return currentX + (topTicks + bottomTicks) * distanceFactor;  
}

double getYDistance() {
  return currentY + (leftTicks + rightTicks) * distanceFactor;
}

double setXPosition() {
  
}

double setYPosition() {
  
}

void moveTo(double x, double y) {
  double initXDistance = x - getXDistance();
  double initYDistance = y - getYDistance();
  double xDistanceLeft, yDistanceLeft;
  
  while (xDistanceLeft > 0.5 && yDistanceLeft > 0.5) {
    analogWrite(8, abs(xDistanceLeft / initXDistance));
    analogWrite(9, abs(xDistanceLeft / initXDistance));
    analogWrite(6, abs(yDistanceLeft / initYDistance));
    analogWrite(7, abs(yDistanceLeft / initYDistance));
    xDistanceLeft = x - getXDistance();
    yDistanceLeft = y - getYDistance();
  }
}

void moveLeftRight(int dist,int dir) { //Distance is in mm Direction is positve is Left negative is Right. Abs value of direction is speed
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);
   
  if (dir > 0) {
    analogWrite(8,abs(dir));
    digitalWrite(9,LOW);
     
  } else {
    analogWrite(9,abs(dir));
    digitalWrite(8,LOW);
    
  }
  
//  while(distanceLR <= dist/1.7);
//  distanceLR = 0;
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);
}

bool isLit() {
  bool isLit = false;
  if (analogRead(0) > 100) {
    isLit = true;
  }
  return isLit;
}

// Modify and test this
//void HandleLeftMotorInterruptA() {
//  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
//  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);   // read the input pin
// 
//  // and adjust counter + if A leads B
//  #ifdef LeftEncoderIsReversed
//    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
//  #else
//    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
//  #endif
//}


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

//    ------| TOP |------
//   |                   |
//   -                   -
//  LEFT               RIGHT
//   -                   -
//   |                   |
//   -----| BOTTOM |------

// Ticks since last measurement
volatile int topTicks = 0;
volatile int bottomTicks = 0;
volatile int leftTicks = 0;
volatile int rightTicks = 0;

// Robot dimensions (in cm)
const double yDimension = 42.545;
const double xDimension = 60.80125;

// Number to multiply ticks by to get distance in cm
const double distanceFactor = 0.026943587;

// Factor to get constant motor power for all motors
const double motorFactor = 0.52466368;

// Set these as the coordinates of the initial robot position (in cm)
double currentX = 0;
double currentY = yDimension / 2;

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
  // Reset the gyro
  // Reset the encoders
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
  moveTo(50, 50);
  hailMother();
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
  // Initial x position of robot + the 
  return currentX + ((topTicks + bottomTicks) / 2) * distanceFactor;  
}

double getYDistance() {
  return currentY + ((leftTicks + rightTicks) / 2) * distanceFactor;
}

double setXPosition() {
  
}

double setYPosition() {
  
}

// Position for robot to move to on the coordinate system (in cm). The origin is centered at the robot's center as specified by currentX and currentY.
void moveTo(double x, double y) {
  double initXDistance = x - getXDistance();
  double initYDistance = y - getYDistance();
  double xDistanceLeft = initXDistance;
  double yDistanceLeft = initYDistance;
  
  if (initXDistance > 0 && initYDistance > 0) {
    while (xDistanceLeft > 0.5 && yDistanceLeft > 0.5) {
      // Top
      moveMotor(7, 8, (xDistanceLeft / initXDistance) * motorFactor);
      // Bottom
      moveMotor(9, 10, (xDistanceLeft / initXDistance) * motorFactor);
      // Left
      moveMotor(3, 4, yDistanceLeft / initYDistance);
      // Right
      moveMotor(5, 6, yDistanceLeft / initYDistance);
      xDistanceLeft = x - getXDistance();
      yDistanceLeft = y - getYDistance();
    }
  }
}

// Move a motor at a specified velocity from -1 to 1. Negative velocity indicates backwards movement. The pwmPin should be wired positively and the ground should be wired negatively.
void moveMotor(int pwmPin, int groundPin, double velocity) {
  if (velocity > 0) {
    analogWrite(pwmPin, (int)(abs(velocity) * 255));
    digitalWrite(groundPin, LOW);   
  }
  else if (velocity < 0) {
    analogWrite(groundPin, (int)(abs(velocity) * 255));
    digitalWrite(pwmPin, LOW);
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

void findLight() {
  while (!isLit()) {
    analogWrite(11, 255);
    delay(1000);
  }
  analogWrite(11, 0);
}

void hitLight() {
  analogWrite(12, 255);
  delay(1000);
  analogWrite(12, 0);
}

bool isLit() {
  bool isLit = false;
  if (analogRead(0) > 100) {
    isLit = true;
  }
  return isLit;
}

void hailMother() {
  int hit = 0;
  while (hit < 6) {
    findLight();
    hitLight();
    hit++;
  }
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

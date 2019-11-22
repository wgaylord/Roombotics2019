#include <Servo.h>
#include <Arduino.h>;

// TODO   
// Make the x and y distances for proportional stuff based on encoders (will be more accurate)
// Ensure the x and y directions are negative when they need to be
// Add velocity to rotate function
// Test code
// Organize/document code

// Assign pins
#define communicationPin 1
#define EStopPin 2
#define leftPwm 3
#define leftGround 4
#define rightPwm 5
#define rightGround 6
#define topPwm 7
#define topGround 8
#define bottomPwm 9
#define bottomGround 10
#define moverPin 11
#define pusherPin 12
#define topInterrupt 18
#define bottomInterrupt 19
#define leftInterrupt 20
#define rightInterrupt 21
#define limitPin 22
#define startPin 23
#define climbReadyPin 24
#define climbPin 25
#define climbGround 26

// Robot wheel names

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

// Distance between wheels (in cm)
const double xDistWheels = 58.5;
const double yDistWheels = 33.5;

// Number to multiply left/right wheel ticks by to get distance in cm
const double yDistanceFactor = 0.026943587;

// Number to multiply top/bottom wheel ticks by to get distance in cm
const double xDistanceFactor = 0.50982475;

// Factor to get constant motor power for all motors
const double motorFactor = 0.52466368;

// Set these as the coordinates of the initial robot position (in cm)
double currentX = 0;
double currentY = yDimension / 2;
double currentAngle = 0;

// Define the servos. Servo mover in continuous mode, servo pusher in servo mode.
Servo mover, pusher;

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

//TODO - Add stopping of the mvoing  button pusher.
//ESTOP - Emergancy Stop. Stops the whole bot then forces into a do nothing loop. Must power cycle to restart.
void ESTOP(){
  digitalWrite(topPwm,LOW);
  digitalWrite(topGround,LOW);
  digitalWrite(bottomPwm,LOW);
  digitalWrite(bottomGround,LOW);
  digitalWrite(leftPwm,LOW);
  digitalWrite(leftGround,LOW);
  digitalWrite(rightPwm,LOW);
  digitalWrite(rightGround,LOW);
  mover.write(180);
  digitalWrite(climbPin,LOW);
  digitalWrite(climbGround,LOW);
  while(true){};
}

// Interrupt pins: 2, 3, 18, 19, 20, 21
void setup() {
  // Reset the encoders
  resetEncoders();

  pinMode(topPwm, OUTPUT);
  pinMode(topGround, OUTPUT);
  pinMode(bottomPwm, OUTPUT);
  pinMode(bottomGround, OUTPUT);
  pinMode(leftPwm, OUTPUT);
  pinMode(leftGround, OUTPUT);
  pinMode(rightPwm, OUTPUT);
  pinMode(rightGround, OUTPUT);
  mover.attach(moverPin);
  pusher.attach(pusherPin);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(EStopPin), ESTOP, RISING);
  attachInterrupt(digitalPinToInterrupt(topInterrupt), addTickTop, RISING);
  attachInterrupt(digitalPinToInterrupt(bottomInterrupt), addTickBottom, RISING);
  attachInterrupt(digitalPinToInterrupt(leftInterrupt), addTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightInterrupt), addTickRight, RISING);
  waitForStart();
}

void loop() {
  // Move robot to buttons
  //moveTo(0, 133.985);
    dumbMove(0,133.985*10);
  // Press the buttons (can't remember how many times we can do this)  (first time is 20 then 10 then 5 then only 1)
  hitLights();
  delay(6000); //Give one extra second just incase.
  hitLights();
  delay(6000);
  hitLights();
  // Move robot to climb position
 // moveTo(38.1, 0);
  dumbMove(38.1*10,0);
  rotate(M_PI / 2, 1.0);
  //Climb
  extendLifter();
  waitForClimb(); //Wait till its time to try and climb. Can't touch bar at all till only 30 seconds left. Signal from ESP.
  moveTo(59.3725, 182.88);
  contractLifter();
}

// Returns the current x distance from the robot's starting position in cm.
double getXDistance() {
  return currentX + getXTicks() * xDistanceFactor;
}

double getYDistance() {
  return currentY + (getYTicks() * yDistanceFactor);
}

double getXTicks() {
  return (topTicks + bottomTicks) / 2;
}

double getYTicks() {
  return (leftTicks + rightTicks) / 2;
}

// Position for robot to move to on the coordinate system (in cm). The origin is centered at the robot's center as specified by currentX and currentY.
void moveTo(double x, double y) {
  // Directions for x and y
  int xDirection = 1;
  int yDirection = 1;
  // Initial x distance in cm
  double xInitDist = x - getXDistance();
  Serial.print("xInitDist = ");
  Serial.println(xInitDist);
  // Initial y distance in cm
  double yInitDist = y - getYDistance();
  Serial.print("yInitDist = ");
  Serial.print(yInitDist);
  // Initial distance (c distance) in cm
  double initDist = sqrt(exp(xInitDist) + exp(yInitDist));
  Serial.print("initDist = ");
  Serial.println(initDist);
  // The average angle phi as calculated using the robot's current angle
  double phi = (asin(yInitDist / initDist) + acos(xInitDist / initDist) - (2 * currentAngle)) / 2;
  Serial.print("phi = ");
  Serial.println(phi);
  // Initial x encoder distance left
  double initXTickDist = initDist * cos(phi) / xDistanceFactor;
  Serial.print("initXTickDist = ");
  Serial.println(initXTickDist);
  // Initial y encoder distance left
  double initYTickDist = initDist * sin(phi) / yDistanceFactor;
  Serial.print("initYTickDist = ");
  Serial.println(initYTickDist);
  double xTicksLeft = initXTickDist;
  double yTicksLeft = initYTickDist;

  // Make sure these are ok despite possible slipping
  if (xInitDist < 0) {
    xDirection = -1;
  }
  if (yInitDist < 0) {
    yDirection = -1;
  }

  while ((abs(xTicksLeft) + abs(yTicksLeft)) > 1) {
    moveAnalogThing(topPwm, topGround, xDirection * xTicksLeft / initXTickDist * motorFactor);
    Serial.print("Top speed: ");
    Serial.println(xDirection * xTicksLeft / initXTickDist);
    moveAnalogThing(bottomPwm, bottomGround, xDirection * xTicksLeft / initXTickDist * motorFactor);
    Serial.print("Bottom speed: ");
    Serial.println(xDirection * xTicksLeft / initXTickDist);
    moveAnalogThing(leftPwm, leftGround, yDirection * yTicksLeft / initYTickDist);
    Serial.print("Left speed: ");
    Serial.println(yDirection * yTicksLeft / initYTickDist);
    moveAnalogThing(rightPwm, rightGround, yDirection * yTicksLeft / initYTickDist);
    Serial.print("Right speed: ");
    Serial.println(yDirection * yTicksLeft / initYTickDist);
    // Reset encoders at beginning of method?
    xTicksLeft = initXTickDist - getXTicks();
    yTicksLeft = initYTickDist - getYTicks();
  }
}


void dumbMove(double x,double y){
    if(y > 0){
    analogWrite(3, 180);
    digitalWrite(4, LOW); 
    analogWrite(5, 180);
    digitalWrite(6, LOW); 
    }else{
    analogWrite(leftGround, 180);
    digitalWrite(leftPwm, LOW);
    analogWrite(rightGround, 180);
    digitalWrite(rightPwm, LOW);
    }
    while(leftTicks <= y/3.5){}
    digitalWrite(leftPwm,LOW);
    digitalWrite(rightPwm,LOW);
    digitalWrite(leftGround,LOW);
    digitalWrite(rightGround,LOW);
        if(y > 0){
      analogWrite(topPwm, 255);
    digitalWrite(topGround, LOW); 
    analogWrite(bottomPwm, 255);
    digitalWrite(bottomGround, LOW); 
    }else{
       analogWrite(topGround, 255);
    digitalWrite(topPwm, LOW);
    analogWrite(bottomGround, 255);
    digitalWrite(bottomPwm, LOW);
    }
    while(topTicks <=x/1.7){}
    digitalWrite(topPwm,LOW);
    digitalWrite(bottomPwm,LOW);
    digitalWrite(topGround,LOW);
    digitalWrite(bottomGround,LOW);
    
    resetEncoders();
}


// Rotate the robot without the use of a gyro. Positive is clockwise. Negative is counterclockwise. Angle in radians.
void rotate(double angle, double velocity) {
  double initAngleDist = angle;
  double angleLeft = initAngleDist;
  currentX = getXDistance();
  currentY = getYDistance();
  resetEncoders();
  while (angleLeft > 0.5) {
    moveAnalogThing(topPwm, topGround, -angleLeft / initAngleDist);
    moveAnalogThing(bottomPwm, bottomGround, angleLeft / initAngleDist);
    moveAnalogThing(leftPwm, bottomPwm, -angleLeft / initAngleDist);
    moveAnalogThing(rightPwm, bottomPwm, angleLeft / initAngleDist);
    angleLeft = angle - ((getXDistance() / xDistWheels + getYDistance() / yDistWheels) / 2);
    currentAngle = ((getXDistance() / xDistWheels + getYDistance() / yDistWheels) / 2);
  }
}

// Manually move the robot with a specific speed and angle. Speed is from -1 to 1 (negative indicates opposite direction). Angle is in radians.
void move(double speed, double angle) {
  double xSpeed = speed * cos(angle);
  double ySpeed = speed * sin(angle);
  
  moveAnalogThing(topPwm, topGround, xSpeed * motorFactor);
  moveAnalogThing(bottomPwm, bottomGround, xSpeed * motorFactor);
  moveAnalogThing(leftPwm, leftGround, ySpeed);
  moveAnalogThing(rightPwm, rightGround, ySpeed);
}

// Move a motor at a specified velocity from -1 to 1. Negative velocity indicates backwards movement. The pwmPin should be wired positively and the ground should be wired negatively.
void moveAnalogThing(int pwmPin, int groundPin, double velocity) {
  int speed = (int)(abs(velocity));
  if (velocity > 0) {
    analogWrite(pwmPin, speed * 255);
    digitalWrite(groundPin, LOW);   
  }
  else if (velocity < 0) {
    analogWrite(groundPin, speed * 255);
    digitalWrite(pwmPin, LOW);
  }
}

void moveDigitalThing(int pwmPin, int groundPin, bool direction) {
  if (direction) {
    digitalWrite(pwmPin, HIGH);
  }
  else {
    digitalWrite(pwmPin, LOW);
  }
}

void resetEncoders() {
  topTicks = 0;
  bottomTicks = 0;
  leftTicks = 0;
  rightTicks = 0;
}

boolean switchPressed() {
  if (digitalRead(limitPin)) {
    return true;
  }
  else {
    return false;
  }
}

void hitLights() {
  Serial.println(analogRead(0));
  for (int lightsHit = 0; lightsHit < 6; lightsHit++) {
    while (!isLit()) {
      // Move servo one way
      Serial.print(isLit());
      mover.write(180); //May want to move slower
    }
    // Push the button
    
    mover.write(90); // Stop mover then press the button.
    pusher.write(120);
    delay(500);
    pusher.write(90); //Will have to adjust these for actual angles needed
    while (!switchPressed()) {
      // Move servo the other way
      mover.write(0);
    }
  }
  // Stop servo
  mover.write(90);
}

bool isLit() {
  bool isLit = false;
  if (analogRead(0) > 100) {
    isLit = true;
  }
  return isLit;
}

void extendLifter(){
  digitalWrite(climbPin,HIGH);
  digitalWrite(climbGround,LOW);
}

void contractLifter(){
  digitalWrite(climbPin,LOW);
  digitalWrite(climbGround,HIGH);
}

//Wait until start of match
void waitForStart(){
  while(!digitalRead(startPin)){} 
}

//Wait for 30 seconds left before even trying to climb. 
void waitForClimb(){
  while(!digitalRead(climbReadyPin)){} 
}


#include <Arduino.h>;
#include <digitalWriteFast.h>

// TODO
// Make the x and y distances for proportional stuff based on encoders (will be more accurate)
// Ensure the x and y directions are negative when they need to be
// Add velocity to rotate function
// Test code
// Organize/document code

// Assign pins
#define topPwm 7
#define topGround 8
#define bottomPwm 9
#define bottomGround 10
#define leftPwm 3
#define leftGround 4
#define rightPwm 5
#define rightGround 6

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

// Number to multiply ticks by to get distance in cm
const double distanceFactor = 0.026943587;

// Factor to get constant motor power for all motors
const double motorFactor = 0.52466368;

// Set these as the coordinates of the initial robot position (in cm)
double currentX = 0;
double currentY = yDimension / 2;
double currentAngle = 0;

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
  digitalWrite(topPwm);
  digitalWrite(topGround);
  digitalWrite(bottomPwm);
  digitalWrite(bottomGround);
  digitalWrite(leftPwm);
  digitalWrite(leftGround);
  digitalWrite(rightPwm);
  digitalWrite(rightGround);
  
  while(true){};
}

// Interrupt pins: 2, 3, 18, 19, 20, 21
void setup() {
  // Reset the gyro
  // Reset the encoders
  topTicks = 0;
  bottomTicks = 0;
  leftTicks = 0;
  rightTicks = 0;

  pinMode(topPwm, OUTPUT);
  pinMode(topGround, OUTPUT);
  pinMode(bottomPwm, OUTPUT);
  pinMode(bottomGround, OUTPUT);
  pinMode(leftPwm, OUTPUT);
  pinMode(leftGround, OUTPUT);
  pinMode(rightPwm, OUTPUT);
  pinMode(rightGround, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), ESTOP, RISING);
  attachInterrupt(digitalPinToInterrupt(18), addTickTop, RISING);
  attachInterrupt(digitalPinToInterrupt(19), addTickBottom, RISING);
  attachInterrupt(digitalPinToInterrupt(20), addTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(21), addTickRight, RISING);

}

void loop() {
  // Move robot to buttons
  moveTo(0, 133.985);
  // Press the buttons
  hailMother();
  // Move robot to climb position
  moveTo(38.1, 0);
  rotate(M_PI / 2, 1.0);
  moveTo(59.3725, 182.88);
  climb();
}

// Returns the current x distance from the robot's starting position in cm.
double getXDistance() {
  return currentX + getXTicks() * distanceFactor;
}

double getYDistance() {
  return currentY + (getYTicks() * distanceFactor);
}

// Get the magnitude of the robot's direction
double getCTicks() {
  return sqrt(exp(getXTicks()) + exp(getYTicks()));
}

double getXTicks() {
  return (topTicks + bottomTicks) / 2;
}

double getYTicks() {
  return (leftTicks + rightTicks) / 2;
}

// double getPhiAngle() {
//   return asin(getYTicks() / getCTicks()) + (M_PI - asin(getXTicks() / getCTicks())) / 2;
// }

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
  double initXTickDist = initDist * cos(phi) / distanceFactor;
  Serial.print("initXTickDist = ");
  Serial.println(initXTickDist);
  // Initial y encoder distance left
  double initYTickDist = initDist * sin(phi) / distanceFactor;
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
    xTicksLeft = initXTickDist - getXTicks();
    yTicksLeft = initYTickDist - getYTicks();
  }
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

// Move a motor at a specified velocity from -1 to 1. Negative velocity indicates backwards movement. The pwmPin should be wired positively and the ground should be wired negatively.
void moveAnalogThing(int pwmPin, int groundPin, double velocity) {
  if (velocity > 0) {
    analogWrite(pwmPin, (int)(abs(velocity) * 255));
    digitalWrite(groundPin, LOW);   
  }
  else if (velocity < 0) {
    analogWrite(groundPin, (int)(abs(velocity) * 255));
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

// Make sure it can change direction -- This is a servo only needs 1 pin to control 
void findLight(boolean pin, boolean ground) {
  digitalWrite(11, pin);
  digitalWrite(12, ground);
  while (!isLit()) {
    if (switchPressed()) {
      ground = switchValue(ground);
      pin = switchValue(pin);
    }
    findLight(pin, ground);
  }
}

boolean switchValue(boolean pin) {
  if (pin == 1) {
    return 0;
  }
  else {
    return 1;
  }
}

boolean switchPressed() {
  if (digitalRead(14)) {
    return true;
  }
  else {
    return false;
  }
}

//Use servo library as analogWrite can't control servos? (Two angles one for in one for out?)
void hitLight() {
  analogWrite(13, 255);
  delay(1000);
  analogWrite(13, 0);
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
    findLight(HIGH, LOW);
    hitLight();
    hit++;
  }
}

void climb() {
//  digitalWrite(HIGH);
}

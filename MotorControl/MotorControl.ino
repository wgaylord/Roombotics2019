
//Pin 2 is FrontBack INTerrupt from encoders
//Pin 3 is LeftRight Interrupt from encoders
//Pin 6 and 7 are front back control
//Pin 8 and 9 are Left right control

volatile int distanceFB = 0; 
volatile int distanceLR = 0;



void FB(){
distanceFB +=1;
}

void LR(){
  distanceLR +=1;
}

void setup() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
Serial.begin(9600);
attachInterrupt(digitalPinToInterrupt(2), FB, RISING);
attachInterrupt(digitalPinToInterrupt(3), LR, RISING);


}

void loop() {

  moveForwardBackwards(1024,100);
  delay(1000);
  moveLeftRight(1024,100);
  delay(1000);
  moveForwardBackwards(1024,-100);
  delay(1000);
  moveLeftRight(1024,-100);
  delay(1000);


}

void moveForwardBackwards(int dist,int dir){ //Distance is in mm Direction is positve is forward negative is back. Abs value of direction is speed
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
   
  if(dir > 0){
    analogWrite(6,abs(dir));
    digitalWrite(7,LOW);
     
  }else{
    analogWrite(7,abs(dir));
    digitalWrite(6,LOW);
    
  }
  
  while(distanceFB <= dist/3.5);
  distanceFB = 0;
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
}

void moveLeftRight(int dist,int dir){ //Distance is in mm Direction is positve is Left negative is Right. Abs value of direction is speed
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);
   
  if(dir > 0){
    analogWrite(8,abs(dir));
    digitalWrite(9,LOW);
     
  }else{
    analogWrite(9,abs(dir));
    digitalWrite(8,LOW);
    
  }
  
  while(distanceLR <= dist/1.7);
  distanceLR = 0;
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);
}


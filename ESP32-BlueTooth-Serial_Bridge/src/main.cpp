#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
boolean waiting = false;
hw_timer_t *timer = NULL; //Setup watchdog for if computer stops talking for 500ms


//Pin 4 on ESP is the ESTOP
void IRAM_ATTR resetModule() {
  digitalWrite(4,HIGH);
  
}


//Uncomment Timer code for competition
void setup() {
  Serial.begin(9600);
  SerialBT.begin("RoomBotics-2019-WG"); //Bluetooth device name
  //timer = timerBegin(0, 80, true);                  //timer 0, div 80
  //timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  //timerAlarmWrite(timer, 500 * 1000, false); //set time in us
  //timerAlarmEnable(timer);                          //enable interrupt
}
void loop() {
 while(Serial.available()) {
    SerialBT.write(Serial.read());
 }
 if (SerialBT.available()) {
   Serial.write(SerialBT.read())
  }
  delay(20);
}

// Change to loop for actual competition.  Pin 15 on ESP is the start signal. Pin 2 on ESP is the ready to lift signal.
void loop1() {
 if (SerialBT.available()) {
   int data = SerialBT.read();
   if(data == 'S'){
     digitalWrite(15,HIGH);
   }
   if(data == 'L'){
     digitalWrite(2,HIGH)
   }
   SerialBT.write('0');
  }
}
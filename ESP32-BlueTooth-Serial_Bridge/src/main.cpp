#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
boolean waiting = false;
hw_timer_t *timer = NULL; //Setup watchdog for if computer stops talking for 500ms

unsigned long waiting = 0;

//Pin 4 on ESP is the ESTOP
void IRAM_ATTR resetModule() {
  digitalWrite(4,HIGH);
  
}



void setup() {
  SerialBT.begin("RoomBotics-2019-WG"); //Bluetooth device name
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, 500 * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
  while (!SerialBT.available()) {}
  SerialBT.read();
  SerialBT.write(1);
  waiting = millis();
  digitalWrite(15,HIGH);
  
}
void loop() {

 if (SerialBT.available()) {
   SerialBT.write(SerialBT.read())
  }
  if(millis() - waiting > 122000){
    digitalWrite(2,HIGH)
  }
  delay(100);
}


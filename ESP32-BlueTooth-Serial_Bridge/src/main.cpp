#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
boolean waiting = false;
hw_timer_t *timer = NULL; //Setup watchdog for if computer stops talking for 500ms

void IRAM_ATTR resetModule() {
  digitalWrite(0,HIGH);
  
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin("RoomBotics-2019-WG"); //Bluetooth device name
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, 500 * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
}
void loop() {
 while(Serial.available()) {
    SerialBT.write(Serial.read());
 }
 if (SerialBT.available()) {
    int type = SerialBT.read();
    if(type == 1){
      digitalWrite(0,HIGH);
    }
    if(type== 0){
      SerialBT.println(0);
    }else{
    while(SerialBT.available()){
      Serial.write(SerialBT.read());
    }
  }}
  delay(20);
}


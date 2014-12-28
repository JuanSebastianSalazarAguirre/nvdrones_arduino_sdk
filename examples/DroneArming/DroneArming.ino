#include <Avant.h>

//
// Create Avant Transmitter which transmits on HardwareSerial1
// 
Avant myTransmitter(1);

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("Arming ");
  myTransmitter.armDrone();
  delay(10000);
}





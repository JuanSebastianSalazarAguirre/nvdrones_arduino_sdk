#include <Avant.h>

//
// Create Avant Transmitter which transmits on HardwareSerial1
// 
Avant myTransmitter(1);

void setup() {
  Serial.begin(115200);
  myTransmitter.avantSetup().
}

void loop() {
  Serial.println("Arming ");
  myTransmitter.armDrone();
  delay(500);
  myTransmitter.armDrone();  
  Serial.println("Disarming ");
  //myTransmitter.disarmDrone();
  delay(5000);
}





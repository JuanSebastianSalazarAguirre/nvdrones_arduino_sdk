#include <Avant.h>

//
// Create Avant Transmitter which transmits on HardwareSerial1
// 
Avant myTransmitter(1);

void setup() {
  delay(2000);
  myTransmitter = Avant(1);
  myTransmitter.avantSetup().setRudderPin(2);    
  myTransmitter.avantSetup().setThrottlePin(3);    
  myTransmitter.avantSetup().setElevatorPin(4);
  myTransmitter.avantSetup().setAilronPin(5);
}

void loop() {
  delay(10);
  myTransmitter.avantSetup().sendSticks();
}






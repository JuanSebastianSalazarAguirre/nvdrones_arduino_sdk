#include <Avant.h>

Transmitter tx;
void setup() {
  TransmitterConfig conf;
  conf.setElevatorPort(4);  
  conf.setAilronPort(5);
  conf.setThrottlePort(2);
  conf.setRudderPort(3);
  conf.setReceiverPort(10);
  conf.setTransmitterPort(11);
  tx.configure(conf);
  tx.calibrate();
}

void loop() {
  tx.transmitData();
}


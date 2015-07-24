#include <Drone.h>

Drone drone(serialPort2);

unsigned long time = millis();

void setup() {
  Serial.begin(56700);
  drone.initialize();
  
  drone.setFlightMode(15);
}

void loop() {
  unsigned long now = millis();

  if (now > time + 3000) {
    Serial.println("Arming");
    //drone.startTransmitterSupport();
    time = now;
  }
  
  drone.listen();
}

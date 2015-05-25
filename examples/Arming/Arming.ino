#include <Drone.h>

Drone drone(serialPort1);

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
    drone.arm();
    time = now;
  }
  
  drone.listen();
}

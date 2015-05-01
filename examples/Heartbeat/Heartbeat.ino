
#include <Drone.h>

Drone drone(serialPort1);
unsigned long lastPrintln = 0;

void setup() {
  drone.heartbeatLostCallback(uhoh);
  lastPrintln = millis();
}

void loop() {
  drone.listen();
}

void uhoh() {
  if (millis() - lastPrintln > 1000) {
    Serial.println("lost heartbeat");
    lastPrintln = millis();
  }
}

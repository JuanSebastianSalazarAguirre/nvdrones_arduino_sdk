#include <Drone.h>

Drone drone(serialPort1);

void setup() {

}

void loop() {
  // Turn the drone on and off every 3 seconds.
  delay(3000);
  Serial.println("Arming");
  drone.arm();
}

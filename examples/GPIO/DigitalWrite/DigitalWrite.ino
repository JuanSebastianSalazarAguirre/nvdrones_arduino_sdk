#include <Drone.h>

Drone drone(serialPort1);

unsigned long time = millis();
int value = HIGH;
int lastPin = 1;

void setup() {
  Serial.begin(57600);

  drone.initialize();

  for (int i=1; i<=8; i++) {
    drone.pinMode(i, OUTPUT);
  }

}

void loop() {

  unsigned long now = millis();

  if (now > time + 2000) {
    lastPin++;
    if (lastPin == 9) {
      lastPin = 1;
      value = (value == HIGH) ? LOW : HIGH;
    }
    drone.digitalWrite(lastPin, value);
    Serial.print("Wrote pin "); Serial.print(lastPin);
    Serial.print(" to: "); Serial.println(value);
    
    time = now;
  }

  drone.listen();

}

void printFloat(String msg, float f) {
  Serial.print(msg);
  Serial.println(f);
}

#include <Drone.h>

Drone drone(serialPort1);

unsigned long time = millis();

void setup() {
  Serial.begin(57600);
  
  drone.initialize();
}

void loop() {
  unsigned long now = millis();

  if (now > time + 1000) {
    float lat = drone.getLatitudeSync();
    printFloat("latitude: ", lat);

    float lng = drone.getLongitudeSync();
    printFloat("longitude: ", lng);
    
    Serial.println("");

    time = now;
  }

  drone.listen();
}

void printFloat(String msg, float f) {
  Serial.print(msg);
  Serial.println(f, 6);
}

// This Example demonstrates how get GPS data
// from the app extender.

#include <Drone.h>

Drone drone(serialPort1);
                 
int interval = 1000; // Sampling Period in milliseconds
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(57600);

  drone.setLongitudeCallback(printLongitude);
  drone.setLatitudeCallback(printLatitude);
}

void loop() {
  if ((unsigned long)(millis() - previousMillis) >= interval) {
    drone.getLongitude();
    drone.getLatitude();
    previousMillis = millis();
  }
  drone.listen();    // Call this to process incoming data from the drone.
}

void printLongitude(float longitude) {
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
}

void printLatitude(float latitude) {
  Serial.print("Latitude: ");
  Serial.println(latitude, 6);
}

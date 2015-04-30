// This Example demonstrates how get GPS data
// from the app extender.

#include <Drone.h>

Drone drone(serialPort1);
                 
int interval = 1000; // Sampling Period in milliseconds
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(57600);

  drone.latitudeCallback(printLatitude);
  drone.longitudeCallback(printLongitude);

  delay(2000);

  float latitude = drone.getLatitudeSync();
  float longitude = drone.getLongitudeSync();
  float altitude = drone.getAltitudeSync();
  int satellites = drone.getSatellitesSync();
  float speed = drone.getSpeedSync();
  float orientation = drone.getOrientationSync();
  Serial.print("Sync latitude is: "); Serial.println(latitude, 6);
  Serial.print("Sync longitude is: "); Serial.println(longitude, 6);
  Serial.print("Sync altitude is: "); Serial.println(altitude, 6);
  Serial.print("Sync satellites is: "); Serial.println(satellites, 6);
  Serial.print("Sync speed is: "); Serial.println(speed, 6);
  Serial.print("Sync orientation is: "); Serial.println(orientation, 6);

  delay(2000);
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

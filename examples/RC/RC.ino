
#include <Drone.h>

Drone drone(serialPort1);

void setup() {
  Serial.begin(56700);

  delay(1000);

  int aileron = drone.getAileronSync();
  int elevator = drone.getElevatorSync();
  int throttle = drone.getThrottleSync();
  int rudder = drone.getRudderSync();
  int flightMode = drone.getFlightModeSync();

  Serial.print("Sync aileron is: "); Serial.println(aileron);
  Serial.print("Sync elevator is: "); Serial.println(elevator);
  Serial.print("Sync throttle is: "); Serial.println(throttle);
  Serial.print("Sync rudder is: "); Serial.println(rudder);
  Serial.print("Sync flightMode is: "); Serial.println(flightMode);
}

void loop() {

}

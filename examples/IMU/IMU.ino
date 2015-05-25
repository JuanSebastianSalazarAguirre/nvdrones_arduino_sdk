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
    float pitchAngle = drone.getPitchAngleSync();
    printFloat("pitch angle ", pitchAngle);

    float rollAngle = drone.getRollAngleSync();
    printFloat("roll angle ", rollAngle);

    float yawAngle = drone.getYawSync();
    printFloat("yaw angle ", yawAngle);
    
    Serial.println("");

    time = now;
  }

  drone.listen();
}

void printFloat(String msg, float f) {
  Serial.print(msg);
  Serial.println(f);
}

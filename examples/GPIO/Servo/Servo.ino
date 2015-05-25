#include <Drone.h>

Drone drone(serialPort1);

unsigned long time = millis();
const int pin = 1;
const int servoNumber = 1;
int currentAngle = 0;

void setup() {
  Serial.begin(57600);

  drone.initialize();
  drone.attachServo(servoNumber, pin);
}

void loop() {
  unsigned long now = millis();

  if (now > time + 2000) {
    drone.writeServo(servoNumber, currentAngle);
    currentAngle = (currentAngle == 0) ? 180 : 0;
    printInt("writing servo to angle: ", currentAngle);
    time = now;
  }

  drone.listen();
}

void printInt(String msg, int i) {
  Serial.print(msg);
  Serial.println(i);
}

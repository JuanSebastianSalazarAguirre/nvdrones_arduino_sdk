#include <Drone.h>

Drone drone(serialPort1); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)

void setup() {
  Serial.begin(57600);
  delay(3000);             //turn the drone on and off every 5 seconds
  Serial.println("Arming");
  drone.armDrone();     //This will turn the drone on and off
}

void loop() {

}





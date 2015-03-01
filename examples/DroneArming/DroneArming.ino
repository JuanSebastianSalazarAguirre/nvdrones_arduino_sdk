#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)

void setup() {
  delay(1000);
  myTransmitter.armDrone();//This will turn the drone on and off
}

void loop() {
  //Do Somestuff
}





#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)


void setup() {
  myTrans.avantSetup().setElevatorPin(1);  //Samples Elevator Pin from Analog Pin 1
  myTrans.avantSetup().setAilronPin(2);   //Samples Ailron Pin from Analog Pin 2
  myTrans.avantSetup().setRudderPin(3);    //Samples Rudder Pin from Analog Pin 3
  myTrans.avantSetup().setThrottlePin(4);  //Samples Throttle Pin from Analog Pin 4
}



void loop() {
  myTrans.avantSetup().sendSticks();  //This will sample all of the Stick Values
                                     //and send it to the drone
  delay(10);
}





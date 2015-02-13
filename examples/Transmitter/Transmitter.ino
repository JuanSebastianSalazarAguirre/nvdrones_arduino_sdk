#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)


void setup() {
  myTrans.transmitter().setElevatorPin(1);  //Samples Elevator Pin from digital Pin 1
  myTrans.transmitter().setAileronPin(2);   //Samples Ailron Pin from digital Pin 2
  myTrans.transmitter().setRudderPin(3);    //Samples Rudder Pin from digital Pin 3
  myTrans.transmitter().setThrottlePin(4);  //Samples Throttle Pin from digital Pin 4
}



void loop() {
  myTrans.transmitter().sendSticks();  //This will sample all of the Stick Values
                                     //and send it to the drone
  delay(10);
}

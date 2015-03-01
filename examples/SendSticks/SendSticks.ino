/*This example demonstrates how to use RC class to send values
through the PWM channels to the Throttle, Rudder, Elevator, 
Aileron, and Flight Mode.  
*/

#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)


void setup() {
  delay(1000);
  myTrans.RC().setFlightMode(15);  //Set the flight mode to Manual Mode
}



void loop() {
  myTrans.RC().setElevator(analogRead(0));  //read the value on analogPin 0, map it to -100 through 100, send it to Elevator
  myTrans.RC().setAileron(analogRead(1));   //read the value on analogPin 0, map it to -100 through 100, send it to Aileron
  myTrans.RC().setThrottle(analogRead(2));  //read the value on analogPin 0, map it to -100 through 100, send it to Throttle
  myTrans.RC().setRudder(analogRead(3));    //read the value on analogPin 0, map it to -100 through 100, send it to Rudder
  delay(10);  //send these values at a rate of 100HZ 
}

//This example demonstrates how to get 
//the logic state of a GPIO pin from 
//the App Extender and turn on an LED 
//when the logic state is high

#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)


void setup() {
  myTrans.GPIO().digitalReadCallback(LED);  //pass the function to be executed to the 
}                                           //digitalRead callback function
                                            //LED() will get executed everytime it receives
                                            //a digitalRead message
void loop() {                               
 myTrans.GPIO().digitalRead(1);  //request the app extender to return the logical state
                                 //of GPIO pin 1
 delay(1000);    //request GPIO information every second
}


//This is an Example Callback function that the user can pass to avantI2C.readCallback()
//and it will be executed whenever a response containing I2C data is received
void LED(byte test){ 
  if(test == 0)
    digitalWrite(13, LOW);
  else
    digitalWrite(13, HIGH);
}

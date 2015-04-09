//This example demonstrates how to get 
//the logic state of a GPIO pin from 
//the App Extender and turn on an LED 
//when the logic state is high

#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)

int interval = 1000;    //Sampling Period in milliseconds 
unsigned long previousMillis=0;

void setup() {
  myTrans.GPIO().digitalReadCallback(LED, 1);  //pass the function to be executed to the 
}                                           //digitalRead callback function
                                            //LED() will get executed everytime it receives
                                            //a digitalRead message
void loop() {                               
  if ((unsigned long)(millis() - previousMillis) >= interval) {
    myTrans.GPIO().digitalRead(1);  //request the app extender to return the logical state of GPIO pin 1
    previousMillis = millis();
  }
  myTrans.avantResponseHandler().responseHandler();  //call this to process any incoming responses
}


//This is an Example Callback function that the user can pass to avantI2C.readCallback()
//and it will be executed whenever a response containing GPIO data is received
void LED(byte test){ 
  if(test == 0)
    digitalWrite(13, LOW);
  else
    digitalWrite(13, HIGH);
}

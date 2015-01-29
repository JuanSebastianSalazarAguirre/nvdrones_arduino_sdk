#include <Avant.h>

Avant myTrans(0); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)


void setup() {
  myTrans.avantI2C().readCallback(testing);
}

void loop() {
 //do some stuff here
 delay(100);

}


//This is an Example Callback function that the user can pass to avantI2C.readCallback()
//and it will be executed whenever a response containing I2C data is received
void testing(byte test ){ 
  Serial.print(test);
}

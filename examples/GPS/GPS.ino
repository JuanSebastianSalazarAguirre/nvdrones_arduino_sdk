//This Example demonstrates how get GPS data using the pose class
//from the app extender and write it to a serial 


#include <Avant.h>

Avant myTrans(1); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)

void setup() {
  Serial.begin(57600);  //Start the Serial Serial Port
  myTrans.pose().longitudeCallback(printLongitude); //pass the function to be executed whenever longitude data is received
  myTrans.pose().latitudeCallback(printLatitude);   //pass the function to be executed whenever latitude data is received 
}

void loop() {
  delay(2000);
  myTrans.pose().getLongitude();
  myTrans.pose().getLatitude();
}

void printLongitude(float longitude) {
  Serial.print("Longitude: ");
  Serial.println(longitude);
}

void printLatitude(float latitude) {
  Serial.print("Latitude: ");
  Serial.println(latitude);
}







//This Example demonstrates how get GPS data using the pose class
//from the app extender and write it to a serial 


#include <Avant.h>

Avant myTrans(2,3); //create an instance of Avant
                  //0 means Serial, 1 is Serial1, 2 is Serial2, 3 is Serial3
                  //If the Xbee is connected to software serial ports
                  //Specify the RX, TX port like myTrans(rx, tx)
                 
int interval = 1000;      //Sampling Period in milliseconds 
unsigned long previousMillis=0;

void setup() {
  Serial.begin(57600);  //Start the Serial Serial Port
  myTrans.initialize();
  myTrans.pose().longitudeCallback(printLongitude); //pass the function to be executed whenever longitude data is received
  myTrans.pose().latitudeCallback(printLatitude);   //pass the function to be executed whenever latitude data is received 
}

void loop() {
  if ((unsigned long)(millis() - previousMillis) >= interval) {
    myTrans.pose().getLongitude();
    myTrans.pose().getLatitude();
    previousMillis = millis();
  }
  myTrans.avantResponseHandler().responseHandler();    //call this to process and incoming responses
}

void printLongitude(float longitude) {
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
}

void printLatitude(float latitude) {
  Serial.print("Latitude: ");
  Serial.println(latitude, 6);
}







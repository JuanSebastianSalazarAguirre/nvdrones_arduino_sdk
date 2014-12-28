#include <Avant.h>

Avant myTrans(11,12);

void receiveDataFromID1(float data) {
  Serial.print("Data received as ");
  Serial.println(data);
}


void setup() {
  /*myTrans.getSetup().setElevatorPin(1);
  myTrans.getSetup().setElevatorPin(1);
  myTrans.getSetup().setElevatorPin(1);

  myTrans.I2C().setDeviceId(ID1);
  
  //myTrans.I2C().getDevice(ID1).setCallbackFunction(receiveDataFromID1);  
*/
  
  //Serial.println(myTrans.getSetup().getElevatorPin());
  
  myTrans.setCallbackFunction(receiveDataFromID1);
}



void loop() {
  
  //Serial.println(myTrans.getSetup().getElevatorPin());
  myTrans.getAvantRC().sendSticks();  
  //myTrans.I2C().getDevice(ID1).read();  //in Hz  
  myTrans.readData();
  delay(5000);
}





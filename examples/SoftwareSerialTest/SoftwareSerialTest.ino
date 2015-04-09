#include <Avant.h>

Avant mySoftTrans(1);
int reset = 0;

void setup() {
  mySoftTrans.initialize();
  mySoftTrans.armDrone();
  delay(2000);
  mySoftTrans.armDrone();
}

void loop() {

}





#include <Avant.h>
Avant myTransmitter(1);

void printByte(byte dat) {
     Serial.println(dat); 
}
void setup() {
    Serial.begin(115200);
    myTransmitter.avantI2C().readCallback(printByte);
}

void loop() {
    myTransmitter.avantI2C().callbackTest(12);
    delay(200);
}

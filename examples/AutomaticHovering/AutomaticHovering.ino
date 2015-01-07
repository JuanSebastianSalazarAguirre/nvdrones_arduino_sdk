#include <Avant.h>

//
// Create Avant Transmitter which transmits on HardwareSerial1
// 
Avant myTransmitter;

void setup() {
  Serial.begin(115200);
  delay(2000);
 
  myTransmitter = Avant(1);
  delay(1000);  
  Serial.println("--------INSTRUCTIONS--------");
  Serial.println("a = arming");
  Serial.println("d = disarming");  
  Serial.println("w = increase throttle");
  Serial.println("s = decrease throttle"); 
  Serial.println("----------------");
}

void loop() {
    if(Serial.available()) {
        
    }
    delay(10);
}






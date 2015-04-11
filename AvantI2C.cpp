#include "AvantI2C.h"

//**********************************
//AvantI2C Class Implementation
//**********************************
AvantI2C::AvantI2C(){}

AvantI2C::AvantI2C(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantI2C::deviceID(uint8_t ID){
  service->sendPacket(ID, 11, 7);
}
void AvantI2C::beginTransmission(void){
  service->sendPacket((int16_t)0, 11, 8);
}

void AvantI2C::endTransmission(void){
  service->sendPacket((int16_t)0, 11, 4);
}

void AvantI2C::write(uint8_t data){
  service->sendPacket((uint8_t)data, 11, 3);
}

void AvantI2C::read(void){
  service->sendPacket((int16_t)0, 11, 5);
}
void AvantI2C::wireRequest(uint8_t bytes){
  service->sendPacket(bytes, 11, 6);
}

void AvantI2C::readCallback(void (*function)(byte)) {
  (*myCallback).i2cRead = function;
}
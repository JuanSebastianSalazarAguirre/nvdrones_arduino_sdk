#include "I2C.h"

I2C::I2C(){}

I2C::I2C(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void I2C::deviceID(uint8_t ID){
  service->sendPacket(ID, 11, 7);
}
void I2C::beginTransmission(void){
  service->sendPacket((int16_t)0, 11, 8);
}

void I2C::endTransmission(void){
  service->sendPacket((int16_t)0, 11, 4);
}

void I2C::write(uint8_t data){
  service->sendPacket((uint8_t)data, 11, 3);
}

void I2C::read(void){
  service->sendPacket((int16_t)0, 11, 5);
}
void I2C::wireRequest(uint8_t bytes){
  service->sendPacket(bytes, 11, 6);
}

void I2C::readCallback(void (*function)(byte)) {
  (*myCallback).i2cRead = function;
}
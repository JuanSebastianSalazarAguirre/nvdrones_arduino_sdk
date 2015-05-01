#include "I2C.h"
#include "IDs.h"

I2C::I2C(){}

I2C::I2C(SerialIO *_serialIO, Callback *_callback) {
  serialIO = _serialIO;
  callbacks = _callback;
}

void I2C::setDeviceAddress(uint8_t address){
  serialIO->sendPacket(address, resourceID::i2c, actionID::setI2CDeviceAddress);
}

void I2C::wireRequest(uint8_t bytes){
  serialIO->sendPacket(bytes, resourceID::i2c, actionID::i2cWireRequest);
}

void I2C::beginTransmission(){
  serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::beginI2CTransmission);
}

void I2C::endTransmission(){
  serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::endI2CTransmission);
}

void I2C::write(uint8_t data){
  serialIO->sendPacket((uint8_t)data, resourceID::i2c, actionID::writeI2C);
}

void I2C::read(){
  serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::readI2C);
}

void I2C::readCallback(void (*cb)(byte)) {
  callbacks->i2cRead = cb;
}

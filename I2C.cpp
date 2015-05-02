#include "I2C.h"
#include "IDs.h"
#include "Utils.h"

I2C::I2C(){}

I2C::I2C(SerialIO *_serialIO, Callback *_callback, ResponseHandler *_responseHandler) {
  serialIO = _serialIO;
  callbacks = _callback;
  responseHandler = _responseHandler;
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

int16_t I2C::readSync() {
  read();
  return Utils::blockForByteData(resourceID::i2c, actionID::readI2C, responseHandler);
}

void I2C::readCallback(void (*cb)(byte)) {
  callbacks->i2cRead = cb;
}

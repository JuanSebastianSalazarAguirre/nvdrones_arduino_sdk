#include "I2C.h"
#include "IDs.h"
#include "Utils.h"

I2C::I2C(){}

I2C::I2C(SerialIO *_serialIO, Callback *_callback, IncomingPacketReader *_incomingPacketReader) {
  serialIO = _serialIO;
  callbacks = _callback;
  incomingPacketReader = _incomingPacketReader;
}

void I2C::setDeviceAddress(int16_t address){
  serialIO->sendPacket((uint8_t)address, resourceID::i2c, actionID::setI2CDeviceAddress);
}

void I2C::wireRequest(int16_t quantity){
  serialIO->sendPacket((uint8_t)quantity, resourceID::i2c, actionID::i2cWireRequest);
}

void I2C::beginTransmission(){
  serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::beginI2CTransmission);
}

void I2C::endTransmission(){
  serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::endI2CTransmission);
}

void I2C::write(int16_t data){
  serialIO->sendPacket((uint8_t)data, resourceID::i2c, actionID::writeI2C);
}

void I2C::read(){
  serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::readI2C);
}

int16_t I2C::readSync() {
  read();
  return Utils::blockForByteData(resourceID::i2c, actionID::readI2C, incomingPacketReader);
}

void I2C::readCallback(void (*cb)(uint8_t)) {
  callbacks->i2cRead = cb;
}

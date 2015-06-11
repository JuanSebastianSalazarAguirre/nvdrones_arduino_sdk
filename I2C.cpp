#include "I2C.h"
#include "IDs.h"
#include "Utils.h"

I2C::I2C(){}

I2C::I2C(SerialIO *serialIO, Callback *callback, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callback),
_incomingPacketReader(incomingPacketReader)
{

}

void I2C::setDeviceAddress(int16_t address){
  _serialIO->sendPacket((uint8_t)address, resourceID::i2c, actionID::setI2CDeviceAddress);
}

void I2C::wireRequest(int16_t quantity){
  _serialIO->sendPacket((uint8_t)quantity, resourceID::i2c, actionID::i2cRequestFrom);
}

void I2C::beginTransmission(){
  _serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::beginI2CTransmission);
}

void I2C::endTransmission(){
  _serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::endI2CTransmission);
}

void I2C::write(int16_t data){
  _serialIO->sendPacket((uint8_t)data, resourceID::i2c, actionID::writeI2C);
}

void I2C::read(){
  _serialIO->sendPacket((int16_t)0, resourceID::i2c, actionID::readI2C);
}

int16_t I2C::readSync() {
  read();
  return Utils::blockForByteData(resourceID::i2c, actionID::readI2C, _incomingPacketReader);
}

void I2C::readCallback(void (*cb)(uint8_t)) {
  _callbacks->i2cRead = cb;
}

void I2C::available() {
  _serialIO->sendPacket((uint8_t)0, resourceID::i2c, actionID::i2cAvailable);
}

int16_t I2C::availableSync() {
  available();
  return Utils::blockForIntData(resourceID::i2c, actionID::i2cAvailable, _incomingPacketReader);
}

void I2C::availableCallback(void (*cb)(int16_t)) {
  _callbacks->i2cAvailable = cb;
}

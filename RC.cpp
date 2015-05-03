#include "RC.h"
#include "IDs.h"
#include "Utils.h"

RC::RC() {};

RC::RC(SerialIO *_serialIO, Callback *_callbacks, IncomingPacketReader *_incomingPacketReader) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  incomingPacketReader = _incomingPacketReader;
}

void RC::setRudder(int8_t value){
  serialIO->sendPacket(value, resourceID::rc, actionID::setRudder);
};

void RC::setThrottle(int8_t value){
  serialIO->sendPacket(value, resourceID::rc, actionID::setThrottle);
};

void RC::setElevator(int8_t value){
  serialIO->sendPacket(value, resourceID::rc, actionID::setElevator);
};

void RC::setAileron(int8_t value){
  serialIO->sendPacket(value, resourceID::rc, actionID::setAileron);
};

void RC::setFlightMode(int8_t value){
  serialIO->sendPacket(value, resourceID::rc, actionID::setFlightMode);
};

void RC::getAileron(){
  serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getAileron);
}

void RC::getElevator(){
  serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getElevator);
}

void RC::getThrottle(){
  serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getThrottle);
}

void RC::getRudder(){
  serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getRudder);
}

void RC::getFlightMode(){
  serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getFlightMode);
}

void RC::flightModeCallback(void (*cb)(int16_t)) {
  callbacks->flightMode = cb;
}

void RC::elevatorCallback(void (*cb)(int16_t)) {
  callbacks->elevator = cb;
}

void RC::aileronCallback(void (*cb)(int16_t)) {
  callbacks->aileron = cb;
}

void RC::rudderCallback(void (*cb)(int16_t)) {
  callbacks->rudder = cb;
}

void RC::throttleCallback(void (*cb)(int16_t)) {
  callbacks->throttle = cb;
}

void RC::setAileronElevatorRudderThrottle(uint8_t aileron, uint8_t elevator, uint8_t rudder, uint8_t throttle) {
  // TODO: implement
}

int16_t RC::getAileronSync() {
  getAileron();
  return Utils::blockForIntData(resourceID::rc, actionID::getAileron, incomingPacketReader);
}

int16_t RC::getElevatorSync() {
  getElevator();
  return Utils::blockForIntData(resourceID::rc, actionID::getElevator, incomingPacketReader);
}

int16_t RC::getThrottleSync() {
  getThrottle();
  return Utils::blockForIntData(resourceID::rc, actionID::getThrottle, incomingPacketReader);
}

int16_t RC::getRudderSync() {
  getRudder();
  return Utils::blockForIntData(resourceID::rc, actionID::getRudder, incomingPacketReader);
}

int16_t RC::getFlightModeSync() {
  getFlightMode();
  return Utils::blockForIntData(resourceID::rc, actionID::getFlightMode, incomingPacketReader);
}

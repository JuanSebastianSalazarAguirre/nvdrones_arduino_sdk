#include "RC.h"
#include "IDs.h"
#include "Utils.h"

RC::RC() {};

RC::RC(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callbacks),
_incomingPacketReader(incomingPacketReader)
{

}

void RC::setRudder(int8_t value){
  _serialIO->sendPacket(value, resourceID::rc, actionID::setRudder);
};

void RC::setThrottle(int8_t value){
  _serialIO->sendPacket(value, resourceID::rc, actionID::setThrottle);
};

void RC::setElevator(int8_t value){
  _serialIO->sendPacket(value, resourceID::rc, actionID::setElevator);
};

void RC::setAileron(int8_t value){
  _serialIO->sendPacket(value, resourceID::rc, actionID::setAileron);
};

void RC::setFlightMode(int8_t value){
  _serialIO->sendPacket(value, resourceID::rc, actionID::setFlightMode);
};

void RC::getAileron(){
  _serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getAileron);
}

void RC::getElevator(){
  _serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getElevator);
}

void RC::getThrottle(){
  _serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getThrottle);
}

void RC::getRudder(){
  _serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getRudder);
}

void RC::getFlightMode(){
  _serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getFlightMode);
}

void RC::flightModeCallback(void (*cb)(int16_t)) {
  _callbacks->flightMode = cb;
}

void RC::elevatorCallback(void (*cb)(int16_t)) {
  _callbacks->elevator = cb;
}

void RC::aileronCallback(void (*cb)(int16_t)) {
  _callbacks->aileron = cb;
}

void RC::rudderCallback(void (*cb)(int16_t)) {
  _callbacks->rudder = cb;
}

void RC::throttleCallback(void (*cb)(int16_t)) {
  _callbacks->throttle = cb;
}

void RC::setAileronElevatorRudderThrottle(int16_t aileron, int16_t elevator, int16_t rudder, int16_t throttle) {
  uint8_t *combined = NULL;
  combined[0] = aileron;
  combined[1] = elevator;
  combined[2] = rudder;
  combined[3] = throttle;
  _serialIO->sendPacket(combined, 4, resourceID::rc, actionID::setAileronElevatorRudderThrottle);
}

int16_t RC::getAileronSync() {
  getAileron();
  return (int8_t)Utils::blockForByteData(resourceID::rc, actionID::getAileron, _incomingPacketReader);
}

int16_t RC::getElevatorSync() {
  getElevator();
  return (int8_t)Utils::blockForByteData(resourceID::rc, actionID::getElevator, _incomingPacketReader);
}

int16_t RC::getThrottleSync() {
  getThrottle();
  return (int8_t)Utils::blockForByteData(resourceID::rc, actionID::getThrottle, _incomingPacketReader);
}

int16_t RC::getRudderSync() {
  getRudder();
  return (int8_t)Utils::blockForByteData(resourceID::rc, actionID::getRudder, _incomingPacketReader);
}

int16_t RC::getFlightModeSync() {
  getFlightMode();
  return (int8_t)Utils::blockForByteData(resourceID::rc, actionID::getFlightMode, _incomingPacketReader);
}

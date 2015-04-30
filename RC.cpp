#include "RC.h"
#include "IDs.h"

RC::RC() {};

RC::RC(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  responseHandler = _responseHandler;
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

void RC::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron){
  long data = (long(rudder+100))+ (long(throttle+100) << 8) + (long(elevator+100) << 16) + (long(aileron+100) << 24);
}

int16_t RC::getAileronSync() {
  getAileron();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == resourceID::rc && p.actionID == actionID::getAileron && p.isValid())
      return (int16_t)p.data[0];
  }
  return -1;
}

int16_t RC::getElevatorSync() {
  getElevator();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == resourceID::rc && p.actionID == actionID::getElevator && p.isValid())
      return (int16_t)p.data[0];
  }
  return -1;
}

int16_t RC::getThrottleSync() {
  getThrottle();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == resourceID::rc && p.actionID == actionID::getThrottle && p.isValid())
      return (int16_t)(int8_t)p.data[0];
  }
  return -1;
}

int16_t RC::getRudderSync() {
  getRudder();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == resourceID::rc && p.actionID == actionID::getRudder && p.isValid())
      return (int16_t)p.data[0];
  }
  return -1;
}

int16_t RC::getFlightModeSync() {
  getFlightMode();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == resourceID::rc && p.actionID == actionID::getFlightMode && p.isValid())
      return (int16_t)p.data[0];
  }
  return -1;
}

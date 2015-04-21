#include "RC.h"

RC::RC() {};

RC::RC(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  responseHandler = _responseHandler;
}

void RC::setRudder(int8_t value){
  serialIO->sendPacket(value, 2, 1);
};

void RC::setThrottle(int8_t value){
  serialIO->sendPacket(value, 2, 2);
};

void RC::setElevator(int8_t value){
  serialIO->sendPacket(value, 2, 3);
};

void RC::setAileron(int8_t value){
  serialIO->sendPacket(value, 2, 4);
};

void RC::setFlightMode(int8_t value){
  serialIO->sendPacket(value, 2, 5);
};

void RC::getAileron(){
  serialIO->sendPacket((int8_t)0, 2, 6);
}

void RC::getElevator(){
  serialIO->sendPacket((int8_t)0, 2, 7);
}

void RC::getThrottle(){
  serialIO->sendPacket((int8_t)0, 2, 8);
}

void RC::getRudder(){
  serialIO->sendPacket((int8_t)0, 2, 9);
}

void RC::getFlightMode(){
  serialIO->sendPacket((int8_t)0, 2, 10);
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
    if (p.resourceID == 2 && p.actionID == 6 && p.isValid())
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
    if (p.resourceID == 2 && p.actionID == 7 && p.isValid())
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
    if (p.resourceID == 2 && p.actionID == 8 && p.isValid())
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
    if (p.resourceID == 2 && p.actionID == 9 && p.isValid())
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
    if (p.resourceID == 2 && p.actionID == 10 && p.isValid())
      return (int16_t)p.data[0];
  }
  return -1;
}

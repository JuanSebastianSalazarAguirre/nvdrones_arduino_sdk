#include "RC.h"

RC::RC() {};

RC::RC(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void RC::setRudder(int8_t value){
  service->sendPacket(value, 2, 1);
};

void RC::setThrottle(int8_t value){
  service->sendPacket(value, 2, 2);
};

void RC::setElevator(int8_t value){
  service->sendPacket(value, 2, 3);
};

void RC::setAileron(int8_t value){
  service->sendPacket(value, 2, 4);
};

void RC::setFlightMode(int8_t value){
  service->sendPacket(value, 2, 5);
};

void RC::getAileron(){
  service->sendPacket((int8_t)0, 2, 6);
}

void RC::getElevator(){
  service->sendPacket((int8_t)0, 2, 7);
}

void RC::getThrottle(){
  service->sendPacket((int8_t)0, 2, 8);
}

void RC::getRudder(){
  service->sendPacket((int8_t)0, 2, 9);
}

void RC::getFlightMode(){
  service->sendPacket((int8_t)0, 2, 10);
}

void RC::setFlightModeCallback(void (*cb)(int16_t)) {
  (*myCallback).flightMode = cb;
}

void RC::setElevatorCallback(void (*cb)(int16_t)) {
  (*myCallback).elevator = cb;
}

void RC::setAileronCallback(void (*cb)(int16_t)) {
  (*myCallback).aileron = cb;
}

void RC::setRudderCallback(void (*cb)(int16_t)) {
  (*myCallback).rudder = cb;
}

void RC::setThrottleCallback(void (*cb)(int16_t)) {
  (*myCallback).throttle = cb;
}

void RC::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron){
  long data = (long(rudder+100))+ (long(throttle+100) << 8) + (long(elevator+100) << 16) + (long(aileron+100) << 24);
}
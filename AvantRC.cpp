/*
 * AvantRC.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: amey
 */
#include "AvantRC.h"

// ***********************************************
// AvantRC Class Implementation
// ***********************************************
AvantRC::AvantRC() {};
AvantRC::AvantRC(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantRC::setAileron(int8_t value){
  service->sendPacket(value, 2, 4);
};
void AvantRC::setElevator(int8_t value){
  service->sendPacket(value, 2, 3);
};
void AvantRC::setThrottle(int8_t value){
  service->sendPacket(value, 2, 2);
};
void AvantRC::setRudder(int8_t value){
  service->sendPacket(value, 2, 1);
};
void AvantRC::setFlightMode(int8_t value){
  service->sendPacket(value, 2, 5);
};
void AvantRC::getAileron(){
  service->sendPacket((int16_t)0, 2, 9);
}
void AvantRC::getElevator(){
  service->sendPacket((int16_t)0, 2, 8);
}
void AvantRC::getThrottle(){
  service->sendPacket((int16_t)0, 2, 7);
}
void AvantRC::getRudder(){
  service->sendPacket((int16_t)0, 2, 6);
}
void AvantRC::getFlightMode(){
  service->sendPacket((int16_t)0, 2, 10);
}

void AvantRC::flightModeCallback(void (*function)(byte)) {
  (*myCallback).flightMode = function;
}

void AvantRC::elevatorCallback(void (*function)(byte)) {
  (*myCallback).elevator= function;
}

void AvantRC::aileronCallback(void (*function)(byte)) {
  (*myCallback).aileron = function;
}

void AvantRC::rudderCallback(void (*function)(byte)) {
  (*myCallback).rudder = function;
}

void AvantRC::throttleCallback(void (*function)(byte)) {
  (*myCallback).throttle = function;
}

void AvantRC::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron){
  long data = (long(rudder+100))+ (long(throttle+100) << 8) + (long(elevator+100) << 16) + (long(aileron+100) << 24);
}




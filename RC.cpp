/*
 * RC.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: amey
 */
#include "RC.h"

// ***********************************************
// RC Class Implementation
// ***********************************************
RC::RC() {};
RC::RC(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void RC::setAileron(int8_t value){
  service->sendPacket(value, 2, 4);
};
void RC::setElevator(int8_t value){
  service->sendPacket(value, 2, 3);
};
void RC::setThrottle(int8_t value){
  service->sendPacket(value, 2, 2);
};
void RC::setRudder(int8_t value){
  service->sendPacket(value, 2, 1);
};
void RC::setFlightMode(int8_t value){
  service->sendPacket(value, 2, 5);
};
void RC::getAileron(){
  service->sendPacket((int16_t)0, 2, 9);
}
void RC::getElevator(){
  service->sendPacket((int16_t)0, 2, 8);
}
void RC::getThrottle(){
  service->sendPacket((int16_t)0, 2, 7);
}
void RC::getRudder(){
  service->sendPacket((int16_t)0, 2, 6);
}
void RC::getFlightMode(){
  service->sendPacket((int16_t)0, 2, 10);
}

void RC::flightModeCallback(void (*function)(byte)) {
  (*myCallback).flightMode = function;
}

void RC::elevatorCallback(void (*function)(byte)) {
  (*myCallback).elevator= function;
}

void RC::aileronCallback(void (*function)(byte)) {
  (*myCallback).aileron = function;
}

void RC::rudderCallback(void (*function)(byte)) {
  (*myCallback).rudder = function;
}

void RC::throttleCallback(void (*function)(byte)) {
  (*myCallback).throttle = function;
}

void RC::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron){
  long data = (long(rudder+100))+ (long(throttle+100) << 8) + (long(elevator+100) << 16) + (long(aileron+100) << 24);
}




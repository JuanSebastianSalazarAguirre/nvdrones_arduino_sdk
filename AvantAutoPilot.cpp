
#include "AvantAutoPilot.h"

//*******************************************
//AvantAutoPilot Class Implementation
//*******************************************
AvantAutoPilot::AvantAutoPilot(){}
AvantAutoPilot::AvantAutoPilot(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantAutoPilot::gpsExecute() {
  service->sendPacket((int16_t)0, 3, 1);
}

void AvantAutoPilot::compassExecute() {
  service->sendPacket((int16_t)0, 3, 2);
}

void AvantAutoPilot::setYawError(float error) {
  service->sendPacket(error, 3, 3);
}

void AvantAutoPilot::setThrottleError(float error) {
  service->sendPacket(error, 3, 4);
}

void AvantAutoPilot::setElevatorError(float error) {
  service->sendPacket(error, 3, 5);
}

void AvantAutoPilot::setAileronError(float error) {
  service->sendPacket(error, 3, 6);
}

void AvantAutoPilot::setWaypointLatitude(float latitude) {
  service->sendPacket(latitude, 15, 1);
}

void AvantAutoPilot::setWaypointLongitude(float longitude) {
  service->sendPacket(longitude, 15, 2);
}

void AvantAutoPilot::setWaypointAltitude(float altitude) {
  service->sendPacket(altitude, 15, 3);
}

void AvantAutoPilot::setWaypointOrientation(float orientation) {
  service->sendPacket(orientation, 15, 4);
}

void AvantAutoPilot::setYawKP(float kp) {
  service->sendPacket(kp, 15, 5);
}

void AvantAutoPilot::setYawKD(float kd) {
  service->sendPacket(kd, 15, 6);
}

void AvantAutoPilot::setYawKI(float ki) {
  service->sendPacket(ki, 15, 7);
}

void AvantAutoPilot::setThrottleKP(float kp) {
  service->sendPacket(kp, 15, 8);
}

void AvantAutoPilot::setThrottleKD(float kd) {
  service->sendPacket(kd, 15, 9);
}

void AvantAutoPilot::setThrottleKI(float ki) {
  service->sendPacket(ki, 15, 10);
}

void AvantAutoPilot::setElevatorKP(float kp) {
  service->sendPacket(kp, 15, 11);
}

void AvantAutoPilot::setElevatorKD(float kd) {
  service->sendPacket(kd, 15, 12);
}

void AvantAutoPilot::setElevatorKI(float ki) {
  service->sendPacket(ki, 15, 13);
}

void AvantAutoPilot::setAileronKP(float kp) {
  service->sendPacket(kp, 15, 14);
}

void AvantAutoPilot::setAileronKD(float kd) {
  service->sendPacket(kd, 15, 15);
}

void AvantAutoPilot::setAileronKI(float ki) {
  service->sendPacket(ki, 15, 16);
}

void AvantAutoPilot::getWaypointLatitude(void (*function)(float)) {
  (*myCallback).getWaypointLatitude = function;
  service->sendPacket((int16_t)0, 15, 22);
}

void AvantAutoPilot::getWaypointLongitude(void (*function)(float)) {
  (*myCallback).getWaypointLongitude = function;
  service->sendPacket((int16_t)0, 15, 23);
}

void AvantAutoPilot::getWaypointAltitude(void (*function)(float)) {
  (*myCallback).getWaypointAltitude = function;
  service->sendPacket((int16_t)0, 15, 24);
}

void AvantAutoPilot::getWaypointOrientation(void (*function)(float)) {
  (*myCallback).getWaypointOrientation = function;
  service->sendPacket((int16_t)0, 15, 25);
}

void AvantAutoPilot::getYawKP(void (*function)(float)) {
  (*myCallback).getYawKP = function;
  service->sendPacket((int16_t)0, 15, 26);
}

void AvantAutoPilot::getYawKD(void (*function)(float)) {
  (*myCallback).getYawKD = function;
  service->sendPacket((int16_t)0, 15, 27);
}

void AvantAutoPilot::getYawKI(void (*function)(float)) {
  (*myCallback).getYawKI = function;
  service->sendPacket((int16_t)0, 15, 28);
}

void AvantAutoPilot::getThrottleKP(void (*function)(float)) {
  (*myCallback).getThrottleKP = function;
  service->sendPacket((int16_t)0, 15, 29);
}

void AvantAutoPilot::getThrottleKD(void (*function)(float)) {
  (*myCallback).getThrottleKD = function;
  service->sendPacket((int16_t)0, 15, 30);
}

void AvantAutoPilot::getThrottleKI(void (*function)(float)) {
  (*myCallback).getThrottleKI = function;
  service->sendPacket((int16_t)0, 15, 31);
}

void AvantAutoPilot::getElevatorKP(void (*function)(float)) {
  (*myCallback).getElevatorKP = function;
  service->sendPacket((int16_t)0, 15, 32);
}

void AvantAutoPilot::getElevatorKD(void (*function)(float)) {
  (*myCallback).getElevatorKD = function;
  service->sendPacket((int16_t)0, 15, 33);
}

void AvantAutoPilot::getElevatorKI(void (*function)(float)) {
  (*myCallback).getElevatorKI = function;
  service->sendPacket((int16_t)0, 15, 34);
}

void AvantAutoPilot::getAileronKP(void (*function)(float)) {
  (*myCallback).getAileronKP = function;
  service->sendPacket((int16_t)0, 15, 35);
}

void AvantAutoPilot::getAileronKD(void (*function)(float)) {
  (*myCallback).getAileronKD = function;
  service->sendPacket((int16_t)0, 15, 36);
}

void AvantAutoPilot::getAileronKI(void (*function)(float)) {
  (*myCallback).getAileronKI = function;
  service->sendPacket((int16_t)0, 15, 37);
}

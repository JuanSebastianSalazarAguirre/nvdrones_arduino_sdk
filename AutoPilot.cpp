#include "AutoPilot.h"

AutoPilot::AutoPilot(){}

AutoPilot::AutoPilot(SerialIO *serialIO, Callback *callback) {
  service = serialIO;
  myCallback = callback;
}

void AutoPilot::gpsExecute() {
  service->sendPacket((int16_t)0, 3, 1);
}

void AutoPilot::compassExecute() {
  service->sendPacket((int16_t)0, 3, 2);
}

void AutoPilot::setYawError(float error) {
  service->sendPacket(error, 3, 3);
}

void AutoPilot::setThrottleError(float error) {
  service->sendPacket(error, 3, 4);
}

void AutoPilot::setElevatorError(float error) {
  service->sendPacket(error, 3, 5);
}

void AutoPilot::setAileronError(float error) {
  service->sendPacket(error, 3, 6);
}

void AutoPilot::setWaypointLatitude(float latitude) {
  service->sendPacket(latitude, 15, 1);
}

void AutoPilot::setWaypointLongitude(float longitude) {
  service->sendPacket(longitude, 15, 2);
}

void AutoPilot::setWaypointAltitude(float altitude) {
  service->sendPacket(altitude, 15, 3);
}

void AutoPilot::setWaypointOrientation(float orientation) {
  service->sendPacket(orientation, 15, 4);
}

void AutoPilot::setYawKP(float kp) {
  service->sendPacket(kp, 15, 5);
}

void AutoPilot::setYawKD(float kd) {
  service->sendPacket(kd, 15, 6);
}

void AutoPilot::setYawKI(float ki) {
  service->sendPacket(ki, 15, 7);
}

void AutoPilot::setThrottleKP(float kp) {
  service->sendPacket(kp, 15, 8);
}

void AutoPilot::setThrottleKD(float kd) {
  service->sendPacket(kd, 15, 9);
}

void AutoPilot::setThrottleKI(float ki) {
  service->sendPacket(ki, 15, 10);
}

void AutoPilot::setElevatorKP(float kp) {
  service->sendPacket(kp, 15, 11);
}

void AutoPilot::setElevatorKD(float kd) {
  service->sendPacket(kd, 15, 12);
}

void AutoPilot::setElevatorKI(float ki) {
  service->sendPacket(ki, 15, 13);
}

void AutoPilot::setAileronKP(float kp) {
  service->sendPacket(kp, 15, 14);
}

void AutoPilot::setAileronKD(float kd) {
  service->sendPacket(kd, 15, 15);
}

void AutoPilot::setAileronKI(float ki) {
  service->sendPacket(ki, 15, 16);
}

void AutoPilot::getWaypointLatitude(void (*function)(float)) {
  (*myCallback).getWaypointLatitude = function;
  service->sendPacket((int16_t)0, 15, 22);
}

void AutoPilot::getWaypointLongitude(void (*function)(float)) {
  (*myCallback).getWaypointLongitude = function;
  service->sendPacket((int16_t)0, 15, 23);
}

void AutoPilot::getWaypointAltitude(void (*function)(float)) {
  (*myCallback).getWaypointAltitude = function;
  service->sendPacket((int16_t)0, 15, 24);
}

void AutoPilot::getWaypointOrientation(void (*function)(float)) {
  (*myCallback).getWaypointOrientation = function;
  service->sendPacket((int16_t)0, 15, 25);
}

void AutoPilot::getYawKP(void (*function)(float)) {
  (*myCallback).getYawKP = function;
  service->sendPacket((int16_t)0, 15, 26);
}

void AutoPilot::getYawKD(void (*function)(float)) {
  (*myCallback).getYawKD = function;
  service->sendPacket((int16_t)0, 15, 27);
}

void AutoPilot::getYawKI(void (*function)(float)) {
  (*myCallback).getYawKI = function;
  service->sendPacket((int16_t)0, 15, 28);
}

void AutoPilot::getThrottleKP(void (*function)(float)) {
  (*myCallback).getThrottleKP = function;
  service->sendPacket((int16_t)0, 15, 29);
}

void AutoPilot::getThrottleKD(void (*function)(float)) {
  (*myCallback).getThrottleKD = function;
  service->sendPacket((int16_t)0, 15, 30);
}

void AutoPilot::getThrottleKI(void (*function)(float)) {
  (*myCallback).getThrottleKI = function;
  service->sendPacket((int16_t)0, 15, 31);
}

void AutoPilot::getElevatorKP(void (*function)(float)) {
  (*myCallback).getElevatorKP = function;
  service->sendPacket((int16_t)0, 15, 32);
}

void AutoPilot::getElevatorKD(void (*function)(float)) {
  (*myCallback).getElevatorKD = function;
  service->sendPacket((int16_t)0, 15, 33);
}

void AutoPilot::getElevatorKI(void (*function)(float)) {
  (*myCallback).getElevatorKI = function;
  service->sendPacket((int16_t)0, 15, 34);
}

void AutoPilot::getAileronKP(void (*function)(float)) {
  (*myCallback).getAileronKP = function;
  service->sendPacket((int16_t)0, 15, 35);
}

void AutoPilot::getAileronKD(void (*function)(float)) {
  (*myCallback).getAileronKD = function;
  service->sendPacket((int16_t)0, 15, 36);
}

void AutoPilot::getAileronKI(void (*function)(float)) {
  (*myCallback).getAileronKI = function;
  service->sendPacket((int16_t)0, 15, 37);
}
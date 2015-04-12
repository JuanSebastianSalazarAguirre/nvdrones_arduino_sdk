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

void AutoPilot::getWaypointLatitude() {
  service->sendPacket((int16_t)0, 15, 22);
}

void AutoPilot::getWaypointLongitude() {
  service->sendPacket((int16_t)0, 15, 23);
}

void AutoPilot::getWaypointAltitude() {
  service->sendPacket((int16_t)0, 15, 24);
}

void AutoPilot::getWaypointOrientation() {
  service->sendPacket((int16_t)0, 15, 25);
}

void AutoPilot::getYawKP() {
  service->sendPacket((int16_t)0, 15, 26);
}

void AutoPilot::getYawKD() {
  service->sendPacket((int16_t)0, 15, 27);
}

void AutoPilot::getYawKI() {
  service->sendPacket((int16_t)0, 15, 28);
}

void AutoPilot::getThrottleKP() {
  service->sendPacket((int16_t)0, 15, 29);
}

void AutoPilot::getThrottleKD() {
  service->sendPacket((int16_t)0, 15, 30);
}

void AutoPilot::getThrottleKI() {
  service->sendPacket((int16_t)0, 15, 31);
}

void AutoPilot::getElevatorKP() {
  service->sendPacket((int16_t)0, 15, 32);
}

void AutoPilot::getElevatorKD() {
  service->sendPacket((int16_t)0, 15, 33);
}

void AutoPilot::getElevatorKI() {
  service->sendPacket((int16_t)0, 15, 34);
}

void AutoPilot::getAileronKP() {
  service->sendPacket((int16_t)0, 15, 35);
}

void AutoPilot::getAileronKD() {
  service->sendPacket((int16_t)0, 15, 36);
}

void AutoPilot::getAileronKI() {
  service->sendPacket((int16_t)0, 15, 37);
}

void AutoPilot::setWaypointLatitudeCallback(void (*cb)(float)) {
  myCallback->waypointLatitude = cb;
}

void AutoPilot::setWaypointLongitudeCallback(void (*cb)(float)) {
  myCallback->waypointLongitude = cb;
}

void AutoPilot::setWaypointAltitudeCallback(void (*cb)(float)) {
  myCallback->waypointAltitude = cb;
}

void AutoPilot::setWaypointOrientationCallback(void (*cb)(float)) {
  myCallback->waypointOrientation = cb;
}

void AutoPilot::setYawKPCallback(void (*cb)(float)) {
  myCallback->yawKP = cb;
}

void AutoPilot::setYawKDCallback(void (*cb)(float)) {
  myCallback->yawKD = cb;
}

void AutoPilot::setYawKICallback(void (*cb)(float)) {
  myCallback->yawKI = cb;
}

void AutoPilot::setThrottleKPCallback(void (*cb)(float)) {
  myCallback->throttleKP = cb;
}

void AutoPilot::setThrottleKDCallback(void (*cb)(float)) {
  myCallback->throttleKD = cb;
}

void AutoPilot::setThrottleKICallback(void (*cb)(float)) {
  myCallback->throttleKI = cb;
}

void AutoPilot::setElevatorKPCallback(void (*cb)(float)) {
  myCallback->elevatorKP = cb;
}

void AutoPilot::setElevatorKDCallback(void (*cb)(float)) {
  myCallback->elevatorKD = cb;
}

void AutoPilot::setElevatorKICallback(void (*cb)(float)) {
  myCallback->elevatorKI = cb;
}

void AutoPilot::setAileronKPCallback(void (*cb)(float)) {
  myCallback->aileronKP = cb;
}

void AutoPilot::setAileronKDCallback(void (*cb)(float)) {
  myCallback->aileronKD = cb;
}

void AutoPilot::setAileronKICallback(void (*cb)(float)) {
  myCallback->aileronKI = cb;
}
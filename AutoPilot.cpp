#include "AutoPilot.h"

AutoPilot::AutoPilot(){}

AutoPilot::AutoPilot(SerialIO *_serialIO, Callback *_callbacks) {
  serialIO = _serialIO;
  callbacks = _callbacks;
}

void AutoPilot::gpsExecute() {
  serialIO->sendPacket((int16_t)0, 3, 1);
}

void AutoPilot::compassExecute() {
  serialIO->sendPacket((int16_t)0, 3, 2);
}

void AutoPilot::setYawError(float error) {
  serialIO->sendPacket(error, 3, 3);
}

void AutoPilot::setThrottleError(float error) {
  serialIO->sendPacket(error, 3, 4);
}

void AutoPilot::setElevatorError(float error) {
  serialIO->sendPacket(error, 3, 5);
}

void AutoPilot::setAileronError(float error) {
  serialIO->sendPacket(error, 3, 6);
}

void AutoPilot::setWaypointLatitude(float latitude) {
  serialIO->sendPacket(latitude, 3, 7);
}

void AutoPilot::setWaypointLongitude(float longitude) {
  serialIO->sendPacket(longitude, 3, 8);
}

void AutoPilot::setWaypointAltitude(float altitude) {
  serialIO->sendPacket(altitude, 3, 9);
}

void AutoPilot::setWaypointOrientation(float orientation) {
  serialIO->sendPacket(orientation, 3, 10);
}

void AutoPilot::setYawKP(float kp) {
  serialIO->sendPacket(kp, 3, 11);
}

void AutoPilot::setYawKD(float kd) {
  serialIO->sendPacket(kd, 3, 12);
}

void AutoPilot::setYawKI(float ki) {
  serialIO->sendPacket(ki, 3, 13);
}

void AutoPilot::setThrottleKP(float kp) {
  serialIO->sendPacket(kp, 3, 14);
}

void AutoPilot::setThrottleKD(float kd) {
  serialIO->sendPacket(kd, 3, 15);
}

void AutoPilot::setThrottleKI(float ki) {
  serialIO->sendPacket(ki, 3, 16);
}

void AutoPilot::setElevatorKP(float kp) {
  serialIO->sendPacket(kp, 3, 17);
}

void AutoPilot::setElevatorKD(float kd) {
  serialIO->sendPacket(kd, 3, 18);
}

void AutoPilot::setElevatorKI(float ki) {
  serialIO->sendPacket(ki, 3, 19);
}

void AutoPilot::setAileronKP(float kp) {
  serialIO->sendPacket(kp, 3, 20);
}

void AutoPilot::setAileronKD(float kd) {
  serialIO->sendPacket(kd, 3, 21);
}

void AutoPilot::setAileronKI(float ki) {
  serialIO->sendPacket(ki, 3, 22);
}

void AutoPilot::setYawMax(float max) {
  serialIO->sendPacket(max, 3, 23);
}

void AutoPilot::setThrottleMax(float max) {
  serialIO->sendPacket(max, 3, 24);
}

void AutoPilot::setElevatorMax(float max) {
  serialIO->sendPacket(max, 3, 25);
}

void AutoPilot::setAileronMax(float max) {
  serialIO->sendPacket(max, 3, 26);
}

void AutoPilot::getWaypointLatitude() {
  serialIO->sendPacket((int16_t)0, 3, 27);
}

void AutoPilot::getWaypointLongitude() {
  serialIO->sendPacket((int16_t)0, 3, 28);
}

void AutoPilot::getWaypointAltitude() {
  serialIO->sendPacket((int16_t)0, 3, 29);
}

void AutoPilot::getWaypointOrientation() {
  serialIO->sendPacket((int16_t)0, 3, 30);
}

void AutoPilot::getYawKP() {
  serialIO->sendPacket((int16_t)0, 3, 31);
}

void AutoPilot::getYawKD() {
  serialIO->sendPacket((int16_t)0, 3, 32);
}

void AutoPilot::getYawKI() {
  serialIO->sendPacket((int16_t)0, 3, 33);
}

void AutoPilot::getThrottleKP() {
  serialIO->sendPacket((int16_t)0, 3, 34);
}

void AutoPilot::getThrottleKD() {
  serialIO->sendPacket((int16_t)0, 3, 35);
}

void AutoPilot::getThrottleKI() {
  serialIO->sendPacket((int16_t)0, 3, 36);
}

void AutoPilot::getElevatorKP() {
  serialIO->sendPacket((int16_t)0, 3, 37);
}

void AutoPilot::getElevatorKD() {
  serialIO->sendPacket((int16_t)0, 3, 38);
}

void AutoPilot::getElevatorKI() {
  serialIO->sendPacket((int16_t)0, 3, 39);
}

void AutoPilot::getAileronKP() {
  serialIO->sendPacket((int16_t)0, 3, 40);
}

void AutoPilot::getAileronKD() {
  serialIO->sendPacket((int16_t)0, 3, 41);
}

void AutoPilot::getAileronKI() {
  serialIO->sendPacket((int16_t)0, 3, 42);
}

void AutoPilot::getYawMax() {
  serialIO->sendPacket((int16_t)0, 3, 43);
}

void AutoPilot::getThrottleMax() {
  serialIO->sendPacket((int16_t)0, 3, 44);
}

void AutoPilot::getElevatorMax() {
  serialIO->sendPacket((int16_t)0, 3, 45);
}

void AutoPilot::getAileronMax() {
  serialIO->sendPacket((int16_t)0, 3, 46);
}

void AutoPilot::setWaypointLatitudeCallback(void (*cb)(float)) {
  callbacks->waypointLatitude = cb;
}

void AutoPilot::setWaypointLongitudeCallback(void (*cb)(float)) {
  callbacks->waypointLongitude = cb;
}

void AutoPilot::setWaypointAltitudeCallback(void (*cb)(float)) {
  callbacks->waypointAltitude = cb;
}

void AutoPilot::setWaypointOrientationCallback(void (*cb)(float)) {
  callbacks->waypointOrientation = cb;
}

void AutoPilot::setYawKPCallback(void (*cb)(float)) {
  callbacks->yawKP = cb;
}

void AutoPilot::setYawKDCallback(void (*cb)(float)) {
  callbacks->yawKD = cb;
}

void AutoPilot::setYawKICallback(void (*cb)(float)) {
  callbacks->yawKI = cb;
}

void AutoPilot::setThrottleKPCallback(void (*cb)(float)) {
  callbacks->throttleKP = cb;
}

void AutoPilot::setThrottleKDCallback(void (*cb)(float)) {
  callbacks->throttleKD = cb;
}

void AutoPilot::setThrottleKICallback(void (*cb)(float)) {
  callbacks->throttleKI = cb;
}

void AutoPilot::setElevatorKPCallback(void (*cb)(float)) {
  callbacks->elevatorKP = cb;
}

void AutoPilot::setElevatorKDCallback(void (*cb)(float)) {
  callbacks->elevatorKD = cb;
}

void AutoPilot::setElevatorKICallback(void (*cb)(float)) {
  callbacks->elevatorKI = cb;
}

void AutoPilot::setAileronKPCallback(void (*cb)(float)) {
  callbacks->aileronKP = cb;
}

void AutoPilot::setAileronKDCallback(void (*cb)(float)) {
  callbacks->aileronKD = cb;
}

void AutoPilot::setAileronKICallback(void (*cb)(float)) {
  callbacks->aileronKI = cb;
}

void AutoPilot::setYawMaxCallback(void (*cb)(float)) {
  callbacks->yawMax = cb;
}

void AutoPilot::setThrottleMaxCallback(void (*cb)(float)) {
  callbacks->throttleMax = cb;
}

void AutoPilot::setElevatorMaxCallback(void (*cb)(float)) {
  callbacks->elevatorMax = cb;
}

void AutoPilot::setAileronMaxCallback(void (*cb)(float)) {
  callbacks->aileronMax = cb;
}
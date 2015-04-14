#include "Pose.h"

Pose::Pose(){}

Pose::Pose(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void Pose::getGPSData(void) {
  service->sendPacket((int16_t)0, 9, 1);
}

void Pose::getLatitude(void) {
  service->sendPacket((int16_t)0, 9, 2);
}

void Pose::getLongitude(void) {
  service->sendPacket((int16_t)0, 9, 3);
}

void Pose::getAltitude(void ) {
  service->sendPacket((int16_t)0, 9, 4);
}

void Pose::getSatellites(void) {
  service->sendPacket((int16_t)0, 9, 5);
}

void Pose::getSpeed(void) {
  service->sendPacket((int16_t)0, 9, 6);
}

void Pose::getOrientation(void){
  service->sendPacket((int16_t)0, 9, 7);
}

void Pose::setLongitudeCallback(void (*function)(float)) {
  (*myCallback).longitude = function;
}

void Pose::setLatitudeCallback(void (*function)(float)) {
  (*myCallback).latitude = function;
}

void Pose::setAltitudeCallback(void (*function)(float)) {
  (*myCallback).altitude = function;
}

void Pose::setSpeedCallback(void (*function)(float)) {
  (*myCallback).speed = function;
}

void Pose::setSatelliteCallback(void (*function)(byte)) {
  (*myCallback).satellite = function;
}

void Pose::setOrientationCallback(void (*function)(float)) {
  (*myCallback).orientation = function;
}
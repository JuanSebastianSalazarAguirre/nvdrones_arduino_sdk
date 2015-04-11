//********************************************
//AvantPose Class Implementation
//********************************************

#include "AvantPose.h"

AvantPose::AvantPose(){}

AvantPose::AvantPose(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantPose::getGPSData(void) {
  service->sendPacket((int16_t)0, 9, 1);
}

void AvantPose::getLatitude(void) {
  service->sendPacket((int16_t)0, 9, 2);
}

void AvantPose::getLongitude(void) {
  service->sendPacket((int16_t)0, 9, 3);
}

void AvantPose::getAltitude(void ) {
  service->sendPacket((int16_t)0, 9, 4);
}

void AvantPose::getSatellites(void) {
  service->sendPacket((int16_t)0, 9, 5);
}

void AvantPose::getSpeed(void) {
  service->sendPacket((int16_t)0, 9, 6);
}

void AvantPose::getOrientation(void){
  service->sendPacket((int16_t)0, 9, 7);
}

void AvantPose::longitudeCallback(void (*function)(float)) {
  (*myCallback).longitude = function;
}

void AvantPose::latitudeCallback(void (*function)(float)) {
  (*myCallback).latitude = function;
}

void AvantPose::altitudeCallback(void (*function)(float)) {
  (*myCallback).altitude = function;
}

void AvantPose::speedCallback(void (*function)(float)) {
  (*myCallback).speed = function;
}

void AvantPose::satelliteCallback(void (*function)(byte)) {
  (*myCallback).satellite = function;
}

void AvantPose::orientationCallback(void (*function)(float)) {
  (*myCallback).orientation = function;
}

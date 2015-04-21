#include "Pose.h"
#include "Utils.h"

Pose::Pose(){}

Pose::Pose(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  responseHandler = _responseHandler;
}

void Pose::getGPSData(void) {
  serialIO->sendPacket((int16_t)0, 9, 1);
}

void Pose::getLatitude(void) {
  serialIO->sendPacket((int8_t)0, 9, 2);
}

void Pose::getLongitude(void) {
  serialIO->sendPacket((int8_t)0, 9, 3);
}

void Pose::getAltitude(void ) {
  serialIO->sendPacket((int8_t)0, 9, 4);
}

void Pose::getSatellites(void) {
  serialIO->sendPacket((int8_t)0, 9, 5);
}

void Pose::getSpeed(void) {
  serialIO->sendPacket((int8_t)0, 9, 6);
}

void Pose::getOrientation(void){
  serialIO->sendPacket((int8_t)0, 9, 7);
}

void Pose::longitudeCallback(void (*cb)(float)) {
  callbacks->longitude = cb;
}

void Pose::latitudeCallback(void (*cb)(float)) {
  callbacks->latitude = cb;
}

void Pose::altitudeCallback(void (*cb)(float)) {
  callbacks->altitude = cb;
}

void Pose::speedCallback(void (*cb)(float)) {
  callbacks->speed = cb;
}

void Pose::satelliteCallback(void (*cb)(int16_t)) {
  callbacks->satellite = cb;
}

void Pose::orientationCallback(void (*cb)(float)) {
  callbacks->orientation = cb;
}

float Pose::getLatitudeSync() {
  getLatitude();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == 9 && p.actionID == 2 && p.isValid())
      return Utils::dataToFloat(p.data);
  }
  return -1;
}

float Pose::getLongitudeSync() {
  getLongitude();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == 9 && p.actionID == 3 && p.isValid())
      return Utils::dataToFloat(p.data);
  }
  return -1;
}

float Pose::getAltitudeSync() {
  getAltitude();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == 9 && p.actionID == 4 && p.isValid())
      return Utils::dataToFloat(p.data);
  }
  return -1;
}

int16_t Pose::getSatellitesSync() {
  getSatellites();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == 9 && p.actionID == 5 && p.isValid())
      return (int16_t)p.data[0];
  }
  return -1;
}

float Pose::getSpeedSync() {
  getSpeed();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == 9 && p.actionID == 6 && p.isValid())
      return Utils::dataToFloat(p.data);
  }
  return -1;
}

float Pose::getOrientationSync() {
  getOrientation();
  IncomingPacket p(0,0,0,0);
  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    p = responseHandler->tryToReadNextPacket();
    if (p.resourceID == 9 && p.actionID == 7 && p.isValid())
      return Utils::dataToFloat(p.data);
  }
  return -1;
}

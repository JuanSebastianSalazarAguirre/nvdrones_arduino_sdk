#include "Pose.h"
#include "IDs.h"
#include "Utils.h"

Pose::Pose(){}

Pose::Pose(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  responseHandler = _responseHandler;
}

void Pose::getGPSData(void) {
  serialIO->sendPacket((int16_t)0, resourceID::pose, actionID::getAllPose);
}

void Pose::getLatitude(void) {
  serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getLatitude);
}

void Pose::getLongitude(void) {
  serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getLongitude);
}

void Pose::getAltitude(void ) {
  serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getAltitude);
}

void Pose::getSatellites(void) {
  serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getSatellites);
}

void Pose::getSpeed(void) {
  serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getSpeed);
}

void Pose::getOrientation(void){
  serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getYaw);
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
    if (p.resourceID == resourceID::pose && p.actionID == actionID::getLatitude && p.isValid())
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
    if (p.resourceID == resourceID::pose && p.actionID == actionID::getLongitude && p.isValid())
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
    if (p.resourceID == resourceID::pose && p.actionID == actionID::getAltitude && p.isValid())
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
    if (p.resourceID == resourceID::pose && p.actionID == actionID::getSatellites && p.isValid())
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
    if (p.resourceID == resourceID::pose && p.actionID == actionID::getSpeed && p.isValid())
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
    if (p.resourceID == resourceID::pose && p.actionID == actionID::getYaw && p.isValid())
      return Utils::dataToFloat(p.data);
  }
  return -1;
}

#include "Pose.h"
#include "IDs.h"
#include "Utils.h"

Pose::Pose(){}

Pose::Pose(SerialIO *_serialIO, Callback *_callbacks, IncomingPacketReader *_incomingPacketReader) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  incomingPacketReader = _incomingPacketReader;
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
  return Utils::blockForFloatData(resourceID::pose, actionID::getLatitude, incomingPacketReader);
}

float Pose::getLongitudeSync() {
  getLongitude();
  return Utils::blockForFloatData(resourceID::pose, actionID::getLongitude, incomingPacketReader);
}

float Pose::getAltitudeSync() {
  getAltitude();
  return Utils::blockForFloatData(resourceID::pose, actionID::getAltitude, incomingPacketReader);
}

int16_t Pose::getSatellitesSync() {
  getSatellites();
  return Utils::blockForIntData(resourceID::pose, actionID::getSatellites, incomingPacketReader);
}

float Pose::getSpeedSync() {
  getSpeed();
  return Utils::blockForFloatData(resourceID::pose, actionID::getSpeed, incomingPacketReader);
}

float Pose::getOrientationSync() {
  getOrientation();
  return Utils::blockForFloatData(resourceID::pose, actionID::getYaw, incomingPacketReader);
}

#include "Pose.h"
#include "IDs.h"
#include "Utils.h"

Pose::Pose(){}

Pose::Pose(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callbacks),
_incomingPacketReader(incomingPacketReader)
{

}

void Pose::getGPSData(void) {
  _serialIO->sendPacket((int16_t)0, resourceID::pose, actionID::getAllPose);
}

void Pose::getLatitude(void) {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getLatitude);
}

void Pose::getLongitude(void) {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getLongitude);
}

void Pose::getAltitude(void ) {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getAltitude);
}

void Pose::getSatellites(void) {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getSatellites);
}

void Pose::getSpeed(void) {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getSpeed);
}

void Pose::getYaw(void){
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getYaw);
}

void Pose::getPitchAngle() {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getPitchAngle);
}

void Pose::getRollAngle() {
  _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRollAngle);
}

void Pose::longitudeCallback(void (*cb)(float)) {
  _callbacks->longitude = cb;
}

void Pose::latitudeCallback(void (*cb)(float)) {
  _callbacks->latitude = cb;
}

void Pose::altitudeCallback(void (*cb)(float)) {
  _callbacks->altitude = cb;
}

void Pose::speedCallback(void (*cb)(float)) {
  _callbacks->speed = cb;
}

void Pose::satelliteCallback(void (*cb)(int16_t)) {
  _callbacks->satellite = cb;
}

void Pose::yawCallback(void (*cb)(float)) {
  _callbacks->yaw = cb;
}

void Pose::pitchAngleCallback(void (*cb)(float)) {
  _callbacks->pitchAngle = cb;
}

void Pose::rollAngleCallback(void (*cb)(float)) {
  _callbacks->rollAngle = cb;
}

float Pose::getLatitudeSync() {
  getLatitude();
  return Utils::blockForFloatData(resourceID::pose, actionID::getLatitude, _incomingPacketReader);
}

float Pose::getLongitudeSync() {
  getLongitude();
  return Utils::blockForFloatData(resourceID::pose, actionID::getLongitude, _incomingPacketReader);
}

float Pose::getAltitudeSync() {
  getAltitude();
  return Utils::blockForFloatData(resourceID::pose, actionID::getAltitude, _incomingPacketReader);
}

int16_t Pose::getSatellitesSync() {
  getSatellites();
  return Utils::blockForIntData(resourceID::pose, actionID::getSatellites, _incomingPacketReader);
}

float Pose::getSpeedSync() {
  getSpeed();
  return Utils::blockForFloatData(resourceID::pose, actionID::getSpeed, _incomingPacketReader);
}

float Pose::getYawSync() {
  getYaw();
  return Utils::blockForFloatData(resourceID::pose, actionID::getYaw, _incomingPacketReader);
}

float Pose::getPitchAngleSync() {
  getPitchAngle();
  return Utils::blockForFloatData(resourceID::pose, actionID::getPitchAngle, _incomingPacketReader);
}

float Pose::getRollAngleSync() {
  getRollAngle();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRollAngle, _incomingPacketReader);
}

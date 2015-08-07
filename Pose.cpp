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


void Pose::rawAccelXCallback(void (*cb)(float)){ _callbacks->rawAccelX = cb;}
void Pose::rawAccelYCallback(void (*cb)(float)){ _callbacks->rawAccelY = cb;}
void Pose::rawAccelZCallback(void (*cb)(float)){ _callbacks->rawAccelZ = cb;}
void Pose::rawGyroXCallback(void (*cb)(float)){ _callbacks->rawGyroX = cb;}
void Pose::rawGyroYCallback(void (*cb)(float)){ _callbacks->rawGyroY = cb;}
void Pose::rawGyroZCallback(void (*cb)(float)){ _callbacks->rawGyroZ = cb;}
void Pose::rawMagnetometerXCallback(void (*cb)(float)){ _callbacks->rawMagnetometerX = cb;}
void Pose::rawMagnetometerYCallback(void (*cb)(float)){ _callbacks->rawMagnetometerY = cb;}
void Pose::rawMagnetometerZCallback(void (*cb)(float)){ _callbacks->rawMagnetometerZ = cb;}
void Pose::altitudeBarometerCallback(void (*cb)(float)){ _callbacks->altitudeBarometer = cb;}

void Pose::getRawAccelX(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawAccelX);}
void Pose::getRawAccelY(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawAccelY);}
void Pose::getRawAccelZ(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawAccelZ);}
void Pose::getRawGyroX(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawGyroX);}
void Pose::getRawGyroY(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawGyroX);}
void Pose::getRawGyroZ(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawGyroX);}
void Pose::getRawMagnetometerX(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawMagnetometerX);}
void Pose::getRawMagnetometerY(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawMagnetometerX);}
void Pose::getRawMagnetometerZ(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getRawMagnetometerX);}
void Pose::getAltitudeBarometer(){ _serialIO->sendPacket((int8_t)0, resourceID::pose, actionID::getAltitudeBarometer);}

float Pose::getRawAccelXSync(){
  getRawAccelX();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawAccelX, _incomingPacketReader);
}
float Pose::getRawAccelYSync(){
  getRawAccelY();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawAccelY, _incomingPacketReader);
}
float Pose::getRawAccelZSync(){
  getRawAccelZ();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawAccelZ, _incomingPacketReader);
}
float Pose::getRawGyroXSync(){
  getRawGyroX();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawGyroX, _incomingPacketReader);
}
float Pose::getRawGyroYSync(){
  getRawGyroY();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawGyroY, _incomingPacketReader);
}
float Pose::getRawGyroZSync(){
  getRawGyroZ();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawGyroZ, _incomingPacketReader);
}
float Pose::getRawMagnetometerXSync(){
  getRawMagnetometerX();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawMagnetometerX, _incomingPacketReader);
}
float Pose::getRawMagnetometerYSync(){
  getRawMagnetometerY();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawMagnetometerY, _incomingPacketReader);
}
float Pose::getRawMagnetometerZSync(){
  getRawMagnetometerZ();
  return Utils::blockForFloatData(resourceID::pose, actionID::getRawMagnetometerZ, _incomingPacketReader);
}
float Pose::getAltitudeBarometerSync(){
  getAltitudeBarometer();
  return Utils::blockForFloatData(resourceID::pose, actionID::getAltitudeBarometer, _incomingPacketReader);
}
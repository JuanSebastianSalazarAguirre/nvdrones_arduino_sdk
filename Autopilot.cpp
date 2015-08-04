#include "Autopilot.h"
#include "IDs.h"
#include "Utils.h"

Autopilot::Autopilot() {};

Autopilot::Autopilot(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callbacks),
_incomingPacketReader(incomingPacketReader)
{

}

//chooses which autopilot feature to enable
void Autopilot::setModeTakeover(){_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::manualTakeover);}
void Autopilot::setModeIndoorHover(){_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::indoorHover);}
void Autopilot::setModeOutdoorHover(){_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::outdoorHover);}

void Autopilot::setAltitudeKp(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setAltitudeKp);}
void Autopilot::setAltitudeKi(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setAltitudeKi);}
void Autopilot::setAltitudeBase(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setAltitudeBase);}
void Autopilot::setAltitudeReference(float value){_serialIO->sendPacket((float)value, resourceID::autopilot, actionID::setAltitudeReference);} 
void Autopilot::setAltitudeTolerance(float value){_serialIO->sendPacket((float)value, resourceID::autopilot, actionID::setAltitudeTolerance);}

void Autopilot::setXPositionKp(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setXPositionKp);}
void Autopilot::setXPositionKi(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setXPositionKi);}
void Autopilot::setXPositionKd(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setXPositionKd);}
void Autopilot::setXPositionReference(float value){_serialIO->sendPacket((float)value, resourceID::autopilot, actionID::setXPositionReference);} 
void Autopilot::setXPositionTolerance(float value){_serialIO->sendPacket((float)value, resourceID::autopilot, actionID::setXPositionTolerance);}

void Autopilot::setYPositionKp(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setYPositionKp);}
void Autopilot::setYPositionKi(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setYPositionKi);}
void Autopilot::setYPositionKd(int value){_serialIO->sendPacket((int8_t)value, resourceID::autopilot, actionID::setYPositionKd);}
void Autopilot::setYPositionReference(float value){_serialIO->sendPacket((float)value, resourceID::autopilot, actionID::setYPositionReference);} 
void Autopilot::setYPositionTolerance(float value){_serialIO->sendPacket((float)value, resourceID::autopilot, actionID::setYPositionTolerance);}

void Autopilot::altitudeKpCallback(void (*cb)(int16_t)){
	_callbacks->getAltitudeKp = cb;
}

void Autopilot::altitudeKiCallback(void (*cb)(int16_t)){
	_callbacks->getAltitudeKi = cb;
}

void Autopilot::altitudeBaseCallback(void (*cb)(int16_t)){
	_callbacks->getAltitudeBase = cb;
}  

void Autopilot::altitudeReferenceCallback(void (*cb)(float)){
	_callbacks->getAltitudeReference = cb;
}

void Autopilot::altitudeToleranceCallback(void (*cb)(float)){
	_callbacks->getAltitudeTolerance = cb;
}  

void Autopilot::XPositionKpCallback(void (*cb)(int16_t)){
	_callbacks->getXPositionKp = cb;
}

void Autopilot::XPositionKiCallback(void (*cb)(int16_t)){
	_callbacks->getXPositionKi = cb;
}

void Autopilot::XPositionKdCallback(void (*cb)(int16_t)){
	_callbacks->getXPositionKd = cb;
}  

void Autopilot::XPositionReferenceCallback(void (*cb)(float)){
	_callbacks->getXPositionReference = cb;
}

void Autopilot::XPositionToleranceCallback(void (*cb)(float)){
	_callbacks->getXPositionTolerance = cb;
} 

void Autopilot::YPositionKpCallback(void (*cb)(int16_t)){
	_callbacks->getYPositionKp = cb;
}

void Autopilot::YPositionKiCallback(void (*cb)(int16_t)){
	_callbacks->getYPositionKi = cb;
}

void Autopilot::YPositionKdCallback(void (*cb)(int16_t)){
	_callbacks->getYPositionKd = cb;
}  

void Autopilot::YPositionReferenceCallback(void (*cb)(float)){
	_callbacks->getYPositionReference = cb;
}

void Autopilot::YPositionToleranceCallback(void (*cb)(float)){
	_callbacks->getYPositionTolerance = cb;
} 

void Autopilot::takeoverCallback(void (*cb)(float)) {
  _callbacks->takeover = cb;
}

void Autopilot::indoorHoverCallback(void (*cb)(float)) {
  _callbacks->indoorHover = cb;
}

void Autopilot::outdoorHoverCallback(void (*cb)(float)) {
  _callbacks->outdoorHover = cb;
}

void Autopilot::setAutopilotMode(int value){
	Serial.print(" Mode: ");Serial.print(value);
	switch(value){
		case 0:
			Serial.print("taking over");
			setModeTakeover();
			break;
		case 1:
			Serial.print("switching to autopilot");
			setModeIndoorHover();
			break;
		case 2:
			Serial.print("swithing to autopilot");
			setModeOutdoorHover();
			break;
	}
}

void Autopilot::getSonarAltitude(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarAltitude);
}

void Autopilot::getSonarXPosition(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarPositionX);
}

void Autopilot::getSonarYPosition(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarPositionY);
}

void Autopilot::getAltitudeKp(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getAltitudeKp);
}

void Autopilot::getAltitudeKi(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getAltitudeKi);
}
 
void Autopilot::getAltitudeBase(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getAltitudeBase);
}

void Autopilot::getAltitudeReference(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getAltitudeReference);
}
 
void Autopilot::getAltitudeTolerance(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getAltitudeTolerance);
}

void Autopilot::getXPositionKp(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getXPositionKp);
}

void Autopilot::getXPositionKi(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getXPositionKi);
}
 
void Autopilot::getXPositionKd(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getXPositionKd);
}

void Autopilot::getXPositionReference(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getXPositionReference);
}
 
void Autopilot::getXPositionTolerance(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getXPositionTolerance);
}

void Autopilot::getYPositionKp(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getYPositionKp);
}

void Autopilot::getYPositionKi(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getYPositionKi);
}
 
void Autopilot::getYPositionKd(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getYPositionKd);
}

void Autopilot::getYPositionReference(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getYPositionReference);
}
 
void Autopilot::getYPositionTolerance(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getYPositionTolerance);
}

void Autopilot::sonarAltitudeCallback(void  (*cb)(float)){
	_callbacks->sonarAltitude = cb;
}

void Autopilot::sonarXPositionCallback(void (*cb)(float)){
	_callbacks->sonarXPosition = cb;
}

void Autopilot::sonarYPositionCallback(void (*cb)(float)){
	_callbacks->sonarYPosition = cb;
}

int16_t Autopilot::getAltitudeBaseSync(){
	getAltitudeBase();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getAltitudeBase, _incomingPacketReader);
}

int16_t Autopilot::getAltitudeKpSync(){
	getAltitudeKp();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getAltitudeKp, _incomingPacketReader);
}

int16_t Autopilot::getAltitudeKiSync(){
	getAltitudeKi();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getAltitudeKi, _incomingPacketReader);
}

float Autopilot::getAltitudeReferenceSync(){
	getAltitudeReference();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getAltitudeReference, _incomingPacketReader);
}

float Autopilot::getAltitudeToleranceSync(){
	getAltitudeTolerance();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getAltitudeTolerance, _incomingPacketReader);
}

int16_t Autopilot::getXPositionKdSync(){
	getXPositionKd();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getXPositionKd, _incomingPacketReader);
}

int16_t Autopilot::getXPositionKpSync(){
	getXPositionKp();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getXPositionKp, _incomingPacketReader);
}

int16_t Autopilot::getXPositionKiSync(){
	getXPositionKi();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getXPositionKi, _incomingPacketReader);
}

float Autopilot::getXPositionReferenceSync(){
	getXPositionReference();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getXPositionReference, _incomingPacketReader);
}

float Autopilot::getXPositionToleranceSync(){
	getXPositionTolerance();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getXPositionTolerance, _incomingPacketReader);
}

int16_t Autopilot::getYPositionKdSync(){
	getYPositionKd();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getYPositionKd, _incomingPacketReader);
}

int16_t Autopilot::getYPositionKpSync(){
	getYPositionKp();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getYPositionKp, _incomingPacketReader);
}

int16_t Autopilot::getYPositionKiSync(){
	getYPositionKi();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getYPositionKi, _incomingPacketReader);
}

float Autopilot::getYPositionReferenceSync(){
	getYPositionReference();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getYPositionReference, _incomingPacketReader);
}

float Autopilot::getYPositionToleranceSync(){
	getYPositionTolerance();
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getYPositionTolerance, _incomingPacketReader);
}

int16_t Autopilot::getSonarXPositionSync(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarPositionX);
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getSonarPositionX, _incomingPacketReader);
}

int16_t Autopilot::getSonarYPositionSync(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarPositionY);
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getSonarPositionY, _incomingPacketReader);

}

int16_t Autopilot::getSonarAltitudeSync(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarAltitude);
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getSonarAltitude, _incomingPacketReader);

}
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

int16_t Autopilot::getSonarAltitudeSync(){
	//_serialIO->sendPacket((int8_t)0, resourceID::rc, actionID::getElevator);
	//return (int8_t)Utils::blockForByteData(resourceID::rc, actionID::getElevator, _incomingPacketReader);
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarAltitude);
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getSonarAltiude, incomingPacketReader);

}

int16_t Autopilot::getSonarPositionXSync(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarPositionX);
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getSonarPositionX, incomingPacketReader);
}

int16_t Autopilot::getSonarPositionYSync(){
	_serialIO->sendPacket((int8_t)0, resourceID::autopilot, actionID::getSonarPositionY);
	return (int8_t)Utils::blockForByteData(resourceID::autopilot, actionID::getSonarPositionY, incomingPacketReader);

}
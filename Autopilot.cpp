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
	switch(value){
		case 0:
			setModeTakeover();
			break;
		case 1:
			setModeIndoorHover();
			break;
		case 2:
			setModeOutdoorHover();
			break;
		default:
			setModeTakeover();
	}
}
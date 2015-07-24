#include "Utils.h"
#include "TransmitterSupport.h"
#include "IDs.h"
#include "SerialIO.h"

TransmitterSupport::TransmitterSupport(){}

TransmitterSupport::TransmitterSupport(SerialIO *serialIO, Callback *callback, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callback),
_incomingPacketReader(incomingPacketReader)
{

}

void TransmitterSupport::startTransmitterSupport(){
	_serialIO->sendPacket(int8_t(1), resourceID::transmitterSupport, actionID::setTransmitterSupportState);
	Serial.println("starting trans Support");
}

void TransmitterSupport::stopTransmitterSupport() {
	_serialIO->sendPacket(int8_t(0), resourceID::transmitterSupport, actionID::setTransmitterSupportState);
}

void TransmitterSupport::startTransmitterCalibration() {
	_serialIO->sendPacket(int8_t(2), resourceID::transmitterSupport, actionID::setTransmitterSupportState);
}

void TransmitterSupport::stopTransmitterCalibration() {
	_serialIO->sendPacket(int8_t(0), resourceID::transmitterSupport, actionID::setTransmitterSupportState);
}
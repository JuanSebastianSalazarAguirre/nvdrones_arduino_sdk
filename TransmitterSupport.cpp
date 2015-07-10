#include "Utils.h"
#include "TransmitterSupport.h"
#include "IDs.h"

TransmitterSupport::TransmitterSupport(){}

TransmitterSupport::TransmitterSupport(SerialIO *serialIO, Callback *callback, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callback),
_incomingPacketReader(incomingPacketReader)
{

}

void TransmitterSupport::startTransmitterSupport(uint16_t type){

}

void TransmitterSupport::stopTransmitterSupport() {

}

void TransmitterSupport::getTransmitterType() {

}

int16_t TransmitterSupport::getTransmitterTypeSync() {

}

void TransmitterSupport::startTransmitterCalibration() {

}

void TransmitterSupport::stopTransmitterCalibration() {

}
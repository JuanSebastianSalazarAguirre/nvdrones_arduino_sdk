
#include "IncomingPacket.h"

IncomingPacket::IncomingPacket(int _resourceID, int _actionID, uint8_t *_data, int _length):
resourceID(_resourceID),
actionID(_actionID),
data(_data),
length(_length)
{

}

bool IncomingPacket::isEmpty()
{
	return actionID == 0 && resourceID == 0;
}

bool IncomingPacket::isHearbeat() {
  return actionID == -1 && resourceID == -1;
}

const IncomingPacket IncomingPacket::emptyPacket(0,0,0,0);
const IncomingPacket IncomingPacket::heartbeatPacket(-1,-1,0,0);
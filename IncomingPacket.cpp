
#include "IncomingPacket.h"

IncomingPacket::IncomingPacket(int _resourceID, int _actionID, uint8_t *_data, int _length):
resourceID(_resourceID),
actionID(_actionID),
data(_data),
length(_length)
{

}

bool IncomingPacket::isValid()
{
	return actionID != 0 && resourceID != 0;
}
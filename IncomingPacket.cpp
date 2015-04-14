
#include "IncomingPacket.h"

IncomingPacket::IncomingPacket(int _actionID, int _resourceID, uint8_t *_data, int _length):
actionID(_actionID),
resourceID(_resourceID),
data(_data),
length(_length)
{

}

bool IncomingPacket::isValid()
{
	return actionID != 0 && resourceID != 0;
}

#ifndef IncomingPacket_h
#define IncomingPacket_h

#include <inttypes.h>

class IncomingPacket
{
public:
  IncomingPacket(int _actionID, int _resourceID, uint8_t *_data, int _length);

  // TODO: implement destructor
  // ~IncomingPacket();

  int actionID;
  int resourceID;
  uint8_t *data;
  int length;
  bool isValid();
}; 

#endif // IncomingPacket_h
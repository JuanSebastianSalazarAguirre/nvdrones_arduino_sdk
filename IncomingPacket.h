
#ifndef NVIncomingPacket_h
#define NVIncomingPacket_h

#include <inttypes.h>

class IncomingPacket
{
public:
  IncomingPacket(int _resourceID, int _actionID, uint8_t *_data, int _length);

  // TODO: implement destructor
  // ~IncomingPacket();

  int resourceID;
  int actionID;
  uint8_t *data;
  int length;
  bool isEmpty();
  bool isHearbeat();

  static const IncomingPacket heartbeatPacket;
  static const IncomingPacket emptyPacket;
};

#endif // NVIncomingPacket_h

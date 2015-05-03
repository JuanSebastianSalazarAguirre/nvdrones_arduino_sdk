
#ifndef NVIncomingPacketReader_h
#define NVIncomingPacketReader_h

#include "IncomingPacketReader.h"
#include "IncomingPacket.h"
#include "SerialIO.h"

class IncomingPacketReader
{
public:
  IncomingPacketReader();
  IncomingPacketReader(SerialIO *serialIO);
  IncomingPacket read();
private:
  SerialIO *_serialIO;
};

#endif // NVIncomingPacketReader_h

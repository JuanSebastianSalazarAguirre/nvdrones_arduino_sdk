
#ifndef NVUtils_h
#define NVUtils_h

#include <inttypes.h>
#include "IncomingPacket.h"
#include "IncomingPacketReader.h"

namespace Utils
{
  float dataToFloat(uint8_t data[]);
  long  dataToLong(uint8_t data[]);
  int dataToInt(uint8_t data[]);

  uint16_t combine(uint8_t high, uint8_t low);

  int16_t blockForByteData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader);
  int16_t blockForIntData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader);
  int32_t blockForLongData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader);
  float blockForFloatData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader);
  IncomingPacket blockForPacket(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader);
}

#endif // NVUtils_h

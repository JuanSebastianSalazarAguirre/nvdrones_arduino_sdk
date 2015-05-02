
#ifndef Utils_h
#define Utils_h

#include <inttypes.h>
#include "IncomingPacket.h"
#include "ResponseHandler.h"

namespace Utils
{
  float dataToFloat(uint8_t data[]);
  long  dataToLong(uint8_t data[]);
  int dataToInt(uint8_t data[]);

  int16_t blockForByteData(int16_t rID, int16_t aID, ResponseHandler *responseHandler);
  int16_t blockForIntData(int16_t rID, int16_t aID, ResponseHandler *responseHandler);
  int32_t blockForLongData(int16_t rID, int16_t aID, ResponseHandler *responseHandler);
  float blockForFloatData(int16_t rID, int16_t aID, ResponseHandler *responseHandler);
  IncomingPacket blockForPacket(int16_t rID, int16_t aID, ResponseHandler *responseHandler);
}

#endif // Utils_h
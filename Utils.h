
#ifndef Utils_h
#define Utils_h

#import <inttypes.h>

namespace Utils
{
  float dataToFloat(uint8_t data[]);
  long  dataToLong(uint8_t data[]);
  int dataToInt(uint8_t data[]);
}

#endif // Utils_h

#include "Utils.h"

float Utils::dataToFloat(uint8_t data[]) {
  union u_tag {
    uint8_t b[4];
    float data_float;
  } u;
  u.b[0] = uint8_t(data[0]);
  u.b[1] = uint8_t(data[1]);
  u.b[2] = uint8_t(data[2]);
  u.b[3] = uint8_t(data[3]);
  return u.data_float;
}

long Utils::dataToLong(uint8_t data[]) {
  long data_long = 0;
  data_long = data[0] << 8;
  data_long = (data_long + data[1]) << 8;
  data_long = (data_long + data[2]) << 8;
  data_long = data_long + data[3]; 
  return data_long;
}

int Utils::dataToInt(uint8_t data[]) {
 int data_int = 0;
 data_int = uint8_t(data[0]) << 8;
 data_int = data_int + uint8_t(data[1]);
 return data_int; 
}
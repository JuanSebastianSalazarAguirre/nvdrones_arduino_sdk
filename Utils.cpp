
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

uint16_t combine(uint8_t high, uint8_t low) {
  return (high << 8) | (low & 0xff);
}

// NOTE: this returns and int, rather than a byte, in order to return -1.
int16_t Utils::blockForByteData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader) {
  IncomingPacket p = blockForPacket(rID, aID, incomingPacketReader);
  return p.isEmpty() ? -1 : p.data[0];
}

int16_t Utils::blockForIntData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader) {
  IncomingPacket p = blockForPacket(rID, aID, incomingPacketReader);
  return p.isEmpty() ? -1 : dataToInt(p.data);
}

float Utils::blockForFloatData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader) {
  IncomingPacket p = blockForPacket(rID, aID, incomingPacketReader);
  return p.isEmpty() ? -1 : dataToFloat(p.data);
}

int32_t Utils::blockForLongData(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader) {
  IncomingPacket p = blockForPacket(rID, aID, incomingPacketReader);
  return p.isEmpty() ? -1 : dataToLong(p.data);
}

IncomingPacket Utils::blockForPacket(int16_t rID, int16_t aID, IncomingPacketReader *incomingPacketReader) {
  IncomingPacket p(0,0,0,0);
  uint32_t startTime = millis();
  while((millis() - startTime) < 1000) {
    p = incomingPacketReader->read();
    if (p.resourceID == rID && p.actionID == aID) {
      return p;
    }
  }
  return IncomingPacket::emptyPacket;
}
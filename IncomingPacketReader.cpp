
#include "IncomingPacketReader.h"
#include <Arduino.h>

#if NV_DEBUG
#define LOG(x,y) Serial.print(x); Serial.println(y);
#else
#define LOG(x,y)
#endif

const int maxIterations = 250;

IncomingPacketReader::IncomingPacketReader() {}

IncomingPacketReader::IncomingPacketReader(SerialIO *serialIO):
_serialIO(serialIO)
{

}

IncomingPacket IncomingPacketReader::read() {
  if (_serialIO->available() == 0) return IncomingPacket::emptyPacket;

  int16_t startByte = _serialIO->read();
  LOG("start byte: ", startByte);
  if (startByte == '!') return IncomingPacket::heartbeatPacket;
  if (startByte != '$') {
    Serial.println("Invalid start byte");
    return IncomingPacket::emptyPacket;
  }

  int16_t length = _serialIO->multipleRead(maxIterations);
  LOG("length: ", length);
  if (length == -1) {
    Serial.println("Error reading length");
    return IncomingPacket::emptyPacket;
  }
  int16_t resourceID = _serialIO->multipleRead(maxIterations);
  LOG("resourceID: ", resourceID);
  if (resourceID == -1) {
    Serial.println("Error reading resourceID");
    return IncomingPacket::emptyPacket;
  }
  int16_t actionID = _serialIO->multipleRead(maxIterations);
  LOG("actionID: ", actionID);
  if (actionID == -1) {
    Serial.println("Error reading actionID");
    return IncomingPacket::emptyPacket;
  }

  uint8_t data[length];
  for (int i=0; i<length; ++i) {
    data[i] = _serialIO->multipleRead(maxIterations);
    LOG("data: ", data[i]);
  }
  // TODO: add error checking.

  int16_t checksum = _serialIO->multipleRead(maxIterations);
  if (checksum == -1) { /* TODO: Error handling */ }
  LOG("checksum: ", checksum);

  int16_t calculatedSum = length + actionID + resourceID;
  for (int i=0; i<length; ++i) {
    calculatedSum += data[i];
  }
  LOG("calculated checksum: ", calculatedSum%256);

  if (checksum != calculatedSum%256) {
    Serial.println("Failed checksum");
    return IncomingPacket::emptyPacket;
  }

  IncomingPacket result(resourceID, actionID, data, length);
  return result;
}
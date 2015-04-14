#include "ResponseHandler.h"

ResponseHandler::ResponseHandler(){};

ResponseHandler::ResponseHandler(SerialIO *_serialIO, Callback *_callbacks):
serialIO(_serialIO),
callbacks(_callbacks)
{

}

// TODO: remove from ResponseHandler class, just make it static
float ResponseHandler::dataToFloat(byte data[]) {
  union u_tag {
    byte b[4];
    float data_float;
  } u;
  u.b[0] = byte(data[0]);
  u.b[1] = byte(data[1]);
  u.b[2] = byte(data[2]);
  u.b[3] = byte(data[3]);
  return u.data_float;
}

// TODO: remove from ResponseHandler class, just make it static
long ResponseHandler::dataToLong(byte data[]) {
  long data_long = 0;
  data_long = data[0] << 8;
  data_long = (data_long + data[1]) << 8;
  data_long = (data_long + data[2]) << 8;
  data_long = data_long + data[3]; 
  return data_long;
}

void ResponseHandler::listen() {
  while (serialIO->available() > 0) {
    IncomingPacket p = tryToReadNextPacket();
    if (!p.isValid()) { /* TODO: Error handling */ }

    switch (p.resourceID) {
      case 9:
        if (p.actionID == 2) callbacks->longitude(dataToFloat(p.data));
        if (p.actionID == 3) callbacks->latitude(dataToFloat(p.data));
        if (p.actionID == 4) callbacks->altitude(dataToFloat(p.data));
        if (p.actionID == 5) callbacks->satellite(dataToFloat(p.data));
        if (p.actionID == 6) callbacks->speed(dataToFloat(p.data));
        if (p.actionID == 7) callbacks->orientation(dataToFloat(p.data));
    }
  }
}

IncomingPacket ResponseHandler::tryToReadNextPacket() {
  int maxIterations = 250;
  IncomingPacket errorPacket(0,0,0,0);
  if (serialIO->available() == 0) return errorPacket;
  if (serialIO->read() != '$') return errorPacket;

  int16_t length = serialIO->multipleRead(maxIterations);
  if (length == -1) { /* TODO: Error handling */ }
  int16_t actionID = serialIO->multipleRead(maxIterations);
  if (actionID == -1) { /* TODO: Error handling */ }
  int16_t resourceID = serialIO->multipleRead(maxIterations);
  if (resourceID == -1) { /* TODO: Error handling */ }

  uint8_t data[length];
  for (int i=0; i<length; ++i) {
    data[i] = serialIO->multipleRead(maxIterations);
  }

  int16_t checksumRead = serialIO->multipleRead(maxIterations);
  if (checksumRead == 256) { /* TODO: Error handling */ }
  uint8_t checksum = lowByte(checksum);

  int16_t calculatedSum = length + actionID + resourceID;
  for (int i=0; i<length; ++i) {
    calculatedSum += data[i];
  }

  if (checksum != calculatedSum%256) { /* TODO: Error handling */ }

  IncomingPacket result(actionID, resourceID, data, length);
  return result;
}

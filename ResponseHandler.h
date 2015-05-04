#ifndef NVResponseHandler_h
#define NVResponseHandler_h

#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacket.h"
#include "IncomingPacketReader.h"
#include "Vitals.h"

class ResponseHandler
{
private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  Vitals *_vitals;
  IncomingPacketReader *_incomingPacketReader;
public:
  ResponseHandler();
  ResponseHandler(SerialIO *serialIO, IncomingPacketReader *incomingPacketReader, Callback *callbacks, Vitals *vitals);
  void listen();
};

#endif // NVResponseHandler_h
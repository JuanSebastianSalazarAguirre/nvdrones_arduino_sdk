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
    SerialIO *serialIO;
    Callback *callbacks;
    Vitals *vitals;
    IncomingPacketReader *incomingPacketReader;
  public:
    ResponseHandler();
    ResponseHandler(SerialIO *_serialIO, IncomingPacketReader *_incomingPacketReader, Callback *_callbacks, Vitals *_vitals);
    void listen();
};

#endif // NVResponseHandler_h
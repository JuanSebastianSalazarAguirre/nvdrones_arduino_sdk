#ifndef __ArduinoSDK__ResponseHandler__
#define __ArduinoSDK__ResponseHandler__

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

#endif /* defined __ArduinoSDK__ResponseHandler__ */
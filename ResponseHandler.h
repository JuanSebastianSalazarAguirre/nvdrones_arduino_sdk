#ifndef __ArduinoSDK__ResponseHandler__
#define __ArduinoSDK__ResponseHandler__

#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacket.h"
#include "Heartbeat.h"

//\cond
class ResponseHandler
//\endcond
{
  private:
    SerialIO *serialIO;
    Callback *callbacks;
    Heartbeat *heartbeat;
  public:
    ResponseHandler();
    ResponseHandler(SerialIO *_serialIO, Callback *_callbacks, Heartbeat *_heartbeat);
    void listen();
    IncomingPacket tryToReadNextPacket();
};

#endif /* defined __ArduinoSDK__ResponseHandler__ */
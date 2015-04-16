#ifndef __ArduinoSDK__ResponseHandler__
#define __ArduinoSDK__ResponseHandler__

#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacket.h"

//\cond
class ResponseHandler
//\endcond
{
  private:
    SerialIO *serialIO;
    Callback *callbacks;
  public:
    ResponseHandler();
    ResponseHandler(SerialIO *_serialIO, Callback *_callbacks);
    void listen();
    IncomingPacket tryToReadNextPacket();
};

#endif /* defined __ArduinoSDK__ResponseHandler__ */
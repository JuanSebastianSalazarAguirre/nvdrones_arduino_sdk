#ifndef __ArduinoSDK__ResponseHandler__
#define __ArduinoSDK__ResponseHandler__

#include "SerialIO.h"
#include "Callback.h"

//\cond
class ResponseHandler
//\endcond
{
  private:
    SerialIO *service;
    Callback *myCallback;
    float dataToFloat(byte data[]);
    long  dataToLong(byte data[]);
  public:
    ResponseHandler();
    ResponseHandler(SerialIO *rcTservice, Callback *callback);
    void responseHandler();
    void callbackTest(byte test){
    myCallback->i2cRead(test);
    }   
};

#endif /* defined __ArduinoSDK__ResponseHandler__ */
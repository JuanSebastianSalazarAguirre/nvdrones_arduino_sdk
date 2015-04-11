#ifndef __ArduinoSDK__AvantResponseHandler__
#define __ArduinoSDK__AvantResponseHandler__

#include "SerialIO.h"
#include "Callback.h"

//\cond
class AvantResponseHandler
//\endcond
{
  private:
    SerialIO *service;
    Callback *myCallback;
    float dataToFloat(byte data[]);
    long  dataToLong(byte data[]);
  public:
    AvantResponseHandler();
    AvantResponseHandler(SerialIO *rcTservice, Callback *callback);
    void responseHandler();
    void callbackTest(byte test){
    myCallback->i2cRead(test);
    }   
};

#endif 
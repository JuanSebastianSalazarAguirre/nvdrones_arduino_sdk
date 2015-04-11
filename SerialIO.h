#ifndef __ArduinoSDK__SerialIO__
#define __ArduinoSDK__SerialIO__

#include <Arduino.h>
#include "SoftwareSerial.h"

// TODO: Description
enum SerialPort {
  serialPort0,
  serialPort1,
  serialPort2,
  serialPort3,
  swSerialPort
};


//\cond
class SerialIO
//\endcond
{
  friend class ResponseHandler;
  private:
    SerialPort selectedSerialPort;


  public:
    //TODO: change it to initialize function
    SoftwareSerial softwareSerial;
    SerialIO();

    /**
    TODO

    @param txPin
    @param rxPin
    */
    SerialIO(int txPin , int rxPin);

    /**
    TODO

    @param hwSerialCode
    */
    SerialIO(SerialPort serialPort);

    /**
    TODO

    @param data
    */
    void write(uint8_t data);

    /**
    Method description
    */
    uint8_t serialRead();

    /**
    Method description
    */
    bool serialAvailable();


    void sendPacket(int16_t data, uint8_t resourceID, uint8_t actionID);

    void sendPacket(uint8_t data, uint8_t resourceID, uint8_t actionID);

    void sendPacket(int8_t data, uint8_t resourceID, uint8_t actionID);

    void sendPacket(float data, uint8_t resourceID, uint8_t actionID);

    //TODO: REMOVE // DEBUG
    /**
    Method description
    */
    void print(String data);

    /**
    Method description
    */
    void readBytes(char *buffer, int bytesToRead);
};

#endif /* defined __ArduinoSDK__SerialIO__ */
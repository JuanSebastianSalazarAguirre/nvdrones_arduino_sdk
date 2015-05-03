
#ifndef NVSerialIO_h
#define NVSerialIO_h

#include <inttypes.h>
#include "SoftwareSerial.h"

// TODO: Description
enum SerialPort {
  serialPort0,
  serialPort1,
  serialPort2,
  serialPort3,
  swSerialPort
};

class SerialIO
{
  private:
    SerialPort selectedSerialPort;
  public:

    SerialIO();
    SerialIO(int txPin , int rxPin);
    SerialIO(SerialPort serialPort);

    SoftwareSerial softwareSerial;

    void write(uint8_t data);
    int16_t read();
    int16_t multipleRead(uint16_t count);
    bool available();
    void sendPacket(int16_t data, uint8_t resourceID, uint8_t actionID);
    void sendPacket(uint8_t data, uint8_t resourceID, uint8_t actionID);
    void sendPacket(int8_t data, uint8_t resourceID, uint8_t actionID);
    void sendPacket(float data, uint8_t resourceID, uint8_t actionID);
    void print(String data);
};

#endif // NVSerialIO_h

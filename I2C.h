
#ifndef NVI2C_h
#define NVI2C_h

#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacketReader.h"

class I2C
/**

 This class allow you to gain access to the I2C serial bus, allowing you to communicate with
 sensors and actuators that you connected to the NVextender.
 
 # Example

 @htmlonly
    <script src="https://gist.github.com/davidgatti/790c8fdcd27385b12043.js"></script>
 @endhtmlonly
 
 */
{
private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  IncomingPacketReader *_incomingPacketReader;
public:
  I2C();
  I2C(SerialIO *serialIO, Callback *callback, IncomingPacketReader *incomingPacketReader);

  void setDeviceAddress(int16_t address);
  void wireRequest(int16_t quantity);
  void beginTransmission();
  void endTransmission();
  void write(int16_t data);
  void read();
  int16_t readSync();
  void readCallback(void (*cb)(uint8_t));
  void available();
  int16_t availableSync();
  void availableCallback(void (*cb)(int16_t));
};

#endif // NVI2C_h

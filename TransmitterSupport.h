
#ifndef TRANSMITTERSUPPORT_h
#define TRANSMITTERSUPPORT_h

#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacketReader.h"

class TransmitterSupport
/**

 This class allow you to turn the Transmitter support service on and off, as 
 Well as configure the recievier type i.e. PWM or Spectrum
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
  TransmitterSupport();
  TransmitterSupport(SerialIO *serialIO, Callback *callback, IncomingPacketReader *incomingPacketReader);

  void startTransmitterSupport(uint8_t type);
  void stopTransmitterSupport();
  void getTransmitterType();
  int16_t getTransmitterTypeSync();
  void startTransmitterCalibration();
  void stopTransmitterCalibration();
};

#endif // TRANSMITTERSUPPORT_h
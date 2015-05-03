
#ifndef Vital_h
#define Vital_h

#include <inttypes.h>
#include "SerialIO.h"
#include "Callback.h"
#include "IncomingPacketReader.h"

class Vitals {
public:
  Vitals();
  Vitals(SerialIO *serialIO, IncomingPacketReader *incomingPacketReader, Callback *callbacks);

  void getVoltage();
  int16_t getVoltageSync();
  void voltageCallback(void (*cb)(int16_t));

  void getSignalStrength();
  int16_t getSignalStrengthSync();
  void signalStrengthCallback(void (*cb)(int16_t));

  void tick();
  void receiveHeartbeat();
  void heartbeatLostCallback(void (*cb)(void));

  void setErrorHandler(void (*cb)(int16_t));

private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  IncomingPacketReader *_incomingPacketReader;

  uint32_t _lastHeartbeatSent;
  uint32_t _lastHearbeatReceived;

  void _sendHeartbeat();

  static const uint32_t _heartbeatInterval = 1000;
  static const uint32_t _allowableHeartbeatSilencePeriod = 5000; // TODO: should this be settable?  
};

#endif // Vital_h

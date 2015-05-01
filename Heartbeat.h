
#ifndef Heartbeat_h
#define Heartbeat_h

#include "SerialIO.h"
#include "Callback.h"

class Heartbeat {
public:
  Heartbeat();
  Heartbeat(SerialIO *serialIO, Callback *callbacks);
  void receive();
  void tick();
  void heartbeatLostCallback(void (*cb)(void));
private:

  SerialIO *_serialIO;
  Callback *_callbacks;
  uint32_t _lastHeartbeatSent;
  uint32_t _lastHearbeatReceived;
  

  void _send();

  static const uint32_t _heartbeatInterval = 1000;
  static const uint32_t _allowableSilencePeriod = 5000; // TODO: should this be settable?
};

#endif // Heartbeat_h
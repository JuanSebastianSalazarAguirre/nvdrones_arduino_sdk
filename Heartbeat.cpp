
#include "Heartbeat.h"

Heartbeat::Heartbeat() {}

Heartbeat::Heartbeat(SerialIO *serialIO, Callback *callbacks):
_serialIO(serialIO),
_callbacks(callbacks),
_lastHeartbeatSent(0),
_lastHearbeatReceived(0)
{

}

void Heartbeat::receive() {
  _lastHearbeatReceived = millis();
}

void Heartbeat::tick() {
  uint32_t now = millis();

  if (now > _lastHearbeatReceived + _allowableSilencePeriod) {
    _callbacks->heartbeatLost();
  }
  if (now > _lastHeartbeatSent + _heartbeatInterval) _send();
}

void Heartbeat::heartbeatLostCallback(void (*cb)(void)) {
  _callbacks->heartbeatLost = cb;
}

void Heartbeat::_send() {
  _serialIO->write('!');
  _lastHeartbeatSent = millis();
}

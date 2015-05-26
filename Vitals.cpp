
#include "Vitals.h"
#include "IDs.h"
#include "Utils.h"

Vitals::Vitals() {}

Vitals::Vitals(SerialIO *serialIO, IncomingPacketReader *incomingPacketReader, Callback *callbacks):
_serialIO(serialIO),
_callbacks(callbacks),
_incomingPacketReader(incomingPacketReader),
_lastHeartbeatSent(0),
_lastHeartbeatReceived(0),
_lastHeartbeatLostCalledTimestamp(0),
_isConnected(false)
{

}

// Voltage

void Vitals::getVoltage() {
  _serialIO->sendPacket(0, resourceID::vitals, actionID::getVoltage);
}

int16_t Vitals::getVoltageSync() {
  getVoltage();
  return Utils::blockForByteData(resourceID::vitals, actionID::getVoltage, _incomingPacketReader);
}

void Vitals::voltageCallback(void (*cb)(int16_t)) {
  _callbacks->voltage = cb;
}

// Signal

void Vitals::getSignalStrength() {
  _serialIO->sendPacket(0, resourceID::vitals, actionID::getSignalStrength);
}

int16_t Vitals::getSignalStrengthSync() {
  getSignalStrength();
  return Utils::blockForByteData(resourceID::vitals, actionID::getSignalStrength, _incomingPacketReader);
}

void Vitals::signalStrengthCallback(void (*cb)(int16_t)) {
  _callbacks->signalStrength = cb;
}

// Heartbeat

void Vitals::tick() {
  uint32_t now = millis();

  if (now > _lastHeartbeatReceived + _allowableHeartbeatSilencePeriod) {
    _serialIO->mute();
    if (_isConnected) {
      _callbacks->heartbeatLost();
      _isConnected = false;
    }
  }

  if (now > _lastHeartbeatSent + _heartbeatInterval) _sendHeartbeat();
}

void Vitals::receiveHeartbeat() {
  if (!_isConnected) {
    _callbacks->heartbeatFound();
    _isConnected = true;
  }
  _lastHeartbeatReceived = millis();
  _serialIO->unmute();
}

void Vitals::heartbeatLostCallback(void (*cb)(void)) {
  _callbacks->heartbeatLost = cb;
}

void Vitals::heartbeatFoundCallback(void (*cb)(void)) {
  _callbacks->heartbeatFound = cb;
}

void Vitals::_sendHeartbeat() {
  _serialIO->write('!');
  _lastHeartbeatSent = millis();
}

// Error Handler

void Vitals::setErrorHandler(void (*cb)(int16_t)) {
  _callbacks->errorHandler = cb;
}



#include "Callback.h"

#include <Arduino.h>

void defaultErrorHandler(int16_t error) {
  Serial.print("received error code: "); Serial.println(error);
}

void defaultCallback(String name) {
  Serial.println(name + " callback not implemented!");
}

void defaultI2CReadCallback(uint8_t) {
  defaultCallback("i2cRead");
}

void defaultI2CAvailableCallback(int16_t) {
  defaultCallback("i2cAvailable");
}

void defaultLongitudeCallback(float) {
  defaultCallback("longitude");
}

void defaultLatitudeCallback(float) {
  defaultCallback("latitude");
}

void defaultAltitudeCallback(float) {
  defaultCallback("altitude");
}

void defaultSpeedCallback(float) {
  defaultCallback("speed");
}

void defaultSatelliteCallback(int16_t) {
  defaultCallback("satellite");
}

void defaultYawCallback(float) {
  defaultCallback("yaw");
}

void defaultPitchAngleCallback(float) {
  defaultCallback("pitchAngle");
}

void defaultRollAngleCallback(float) {
  defaultCallback("rollAngle");
}

void defaultFlightModeCallback(int16_t) {
  defaultCallback("flightMode");
}

void defaultElevatorCallback(int16_t) {
  defaultCallback("elevator");
}

void defaultAileronCallback(int16_t) {
  defaultCallback("aileron");
}

void defaultThrottleCallback(int16_t) {
  defaultCallback("throttle");
}

void defaultRudderCallback(int16_t) {
  defaultCallback("rudder");
}

void defaultPulseIn1Callback(uint32_t) {
  defaultCallback("pulseIn1");
}

void defaultPulseIn2Callback(uint32_t) {
  defaultCallback("pulseIn2");
}

void defaultPulseIn3Callback(uint32_t) {
  defaultCallback("pulseIn3");
}

void defaultPulseIn4Callback(uint32_t) {
  defaultCallback("pulseIn4");
}

void defaultPulseIn5Callback(uint32_t) {
  defaultCallback("pulseIn5");
}

void defaultPulseIn6Callback(uint32_t) {
  defaultCallback("pulseIn6");
}

void defaultPulseIn7Callback(uint32_t) {
  defaultCallback("pulseIn7");
}

void defaultPulseIn8Callback(uint32_t) {
  defaultCallback("pulseIn8");
}

void defaultPulseIn9Callback(uint32_t) {
  defaultCallback("pulseIn9");
}

void defaultPulseIn10Callback(uint32_t) {
  defaultCallback("pulseIn10");
}

void defaultDigitalRead1Callback(int16_t) {
  defaultCallback("digitalRead1");
}

void defaultDigitalRead2Callback(int16_t) {
  defaultCallback("digitalRead2");
}

void defaultDigitalRead3Callback(int16_t) {
  defaultCallback("digitalRead3");
}

void defaultDigitalRead4Callback(int16_t) {
  defaultCallback("digitalRead4");
}

void defaultDigitalRead5Callback(int16_t) {
  defaultCallback("digitalRead5");
}

void defaultDigitalRead6Callback(int16_t) {
  defaultCallback("digitalRead6");
}

void defaultDigitalRead7Callback(int16_t) {
  defaultCallback("digitalRead7");
}

void defaultDigitalRead8Callback(int16_t) {
  defaultCallback("digitalRead8");
}

void defaultDigitalRead9Callback(int16_t) {
  defaultCallback("digitalRead9");
}

void defaultDigitalRead10Callback(int16_t) {
  defaultCallback("digitalRead10");
}

void defaultAnalogRead1Callback(int16_t) {
  defaultCallback("analogRead1");
}

void defaultAnalogRead2Callback(int16_t) {
  defaultCallback("analogRead2");
}

void defaultAnalogRead3Callback(int16_t) {
  defaultCallback("analogRead3");
}

void defaultAnalogRead4Callback(int16_t) {
  defaultCallback("analogRead4");
}

void defaultInterrupt0Callback(void) {
  defaultCallback("interrupt0");
}

void defaultInterrupt1Callback(void) {
  defaultCallback("interrupt1");
}

void defaultHeartbeatLostCallback() {
  defaultCallback("heartbeatLost");
}

void defaultHeartbeatFoundCallback() {
  defaultCallback("heartbeatFound");
}

void defaultVoltageCallback(int16_t) {
  defaultCallback("voltage");
}

void defaultSignalStrength(int16_t) {
  defaultCallback("signalStrength");
}

void defaultIndoorHoverCallback(float) {
  defaultCallback("switchMode indoor");
}

void defaultOutdoorHoverCallback(float) {
  defaultCallback("switchMode outdoor");
}

void defaultTakeoverCallback(float) {
  defaultCallback("Takeover");
}

Callback::Callback():
i2cRead(&defaultI2CReadCallback),
i2cAvailable(&defaultI2CAvailableCallback),
longitude(&defaultLongitudeCallback),
latitude(&defaultLatitudeCallback),
altitude(&defaultAltitudeCallback),
speed(&defaultSpeedCallback),
satellite(&defaultSatelliteCallback),
yaw(&defaultYawCallback),
pitchAngle(&defaultPitchAngleCallback),
rollAngle(&defaultRollAngleCallback),
flightMode(&defaultFlightModeCallback),
elevator(&defaultElevatorCallback),
aileron(&defaultAileronCallback),
throttle(&defaultThrottleCallback),
rudder(&defaultRudderCallback),
pulseIn1(&defaultPulseIn1Callback),
pulseIn2(&defaultPulseIn2Callback),
pulseIn3(&defaultPulseIn3Callback),
pulseIn4(&defaultPulseIn4Callback),
pulseIn5(&defaultPulseIn5Callback),
pulseIn6(&defaultPulseIn6Callback),
pulseIn7(&defaultPulseIn7Callback),
pulseIn8(&defaultPulseIn8Callback),
pulseIn9(&defaultPulseIn9Callback),
pulseIn10(&defaultPulseIn10Callback),

digitalRead1(&defaultDigitalRead1Callback),
digitalRead2(&defaultDigitalRead2Callback),
digitalRead3(&defaultDigitalRead3Callback),
digitalRead4(&defaultDigitalRead4Callback),
digitalRead5(&defaultDigitalRead5Callback),
digitalRead6(&defaultDigitalRead6Callback),
digitalRead7(&defaultDigitalRead7Callback),
digitalRead8(&defaultDigitalRead8Callback),
digitalRead9(&defaultDigitalRead9Callback),
digitalRead10(&defaultDigitalRead10Callback),

analogRead1(&defaultAnalogRead1Callback),
analogRead2(&defaultAnalogRead2Callback),
analogRead3(&defaultAnalogRead3Callback),
analogRead4(&defaultAnalogRead4Callback),

interrupt0(&defaultInterrupt0Callback),
interrupt1(&defaultInterrupt1Callback),

heartbeatLost(&defaultHeartbeatLostCallback),
heartbeatFound(&defaultHeartbeatFoundCallback),
voltage(&defaultVoltageCallback),
signalStrength(&defaultSignalStrength),

errorHandler(&defaultErrorHandler),

indoorHover(&defaultIndoorHoverCallback),
outdoorHover(&defaultOutdoorHoverCallback),
takeover(&defaultTakeoverCallback)
{

}

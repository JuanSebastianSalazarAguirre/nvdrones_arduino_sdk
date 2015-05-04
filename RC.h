
#ifndef NVRC_h
#define NVRC_h

#include "Callback.h"
#include "SerialIO.h"
#include "IncomingPacketReader.h"

class RC //handles sending values to the PWM/PPM port(s)
/**

 This class allows you to control your drone: set the throttle, rudder, elevator, and aileron channels on the drone.
 Essentially controlling up down left right ect.

*/
{
private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  IncomingPacketReader *_incomingPacketReader;
public:
  RC();
  RC(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader);

  void setAileron(int8_t value);
  void setElevator(int8_t value);
  void setThrottle(int8_t value);
  void setRudder(int8_t value);
  void setFlightMode(int8_t value);
  void getAileron();
  void getElevator();
  void getThrottle();
  void getRudder();
  void getFlightMode();
  void aileronCallback(void (*function)(int16_t));
  void flightModeCallback(void (*function)(int16_t));
  void throttleCallback(void (*function)(int16_t));
  void rudderCallback(void (*function)(int16_t));
  void elevatorCallback(void (*function)(int16_t));
  void setAileronElevatorRudderThrottle(uint8_t aileron, uint8_t elevator, uint8_t rudder, uint8_t throttle);
  int16_t getAileronSync();
  int16_t getElevatorSync();
  int16_t getThrottleSync();
  int16_t getRudderSync();
  int16_t getFlightModeSync();
};

#endif // NVRC_h

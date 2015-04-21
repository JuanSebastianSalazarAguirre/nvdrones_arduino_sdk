#ifndef __ArduinoSDK__RC__
#define __ArduinoSDK__RC__

#include "Callback.h"
#include "SerialIO.h"
#include "ResponseHandler.h"

class RC //handles sending values to the PWM/PPM port(s)
/**

 This class allows you to control your drone: set the throttle, rudder, elevator, and aileron channels on the drone.
 Essentially controlling up down left right ect.

*/
{
private:
  //\cond
  SerialIO *serialIO;
  Callback *callbacks;
  ResponseHandler *responseHandler;
  //\endcond
public:
  //\cond
  RC();

  RC(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler);
  //\endcond

  /**

  Sends a request to the extender to set the Aileron channel to a specific channel between -100 and 100.

  @param value The value to set the Aileron channel to

  */
  void setAileron(int8_t value);

  /**

  Sends a request to the extender to set the Extender channel to a specific channel between -100 and 100.

  @param value The value to set the Extender channel to

  */
  void setElevator(int8_t value);

  /**

  Sends a request to the extender to set the Throttle channel to a specific channel between -100 and 100.

  @param value The value to set the Throttle channel to

  */
  void setThrottle(int8_t value);

  /**

  Sends a request to the extender to set the Rudder channel to a specific channel between -100 and 100.

  @param value The value to set the Rudder channel to

  */
  void setRudder(int8_t value);

  /**

  Sends a request to the extender to set the Flight Mode channel to a specific channel between -100 and 100.

  @param value The value to set the Flight Mode channel to

  */
  void setFlightMode(int8_t value);

  /**

  Sends a request to the extender to reply with message containing the current Aileron value.

  */
  void getAileron();

  /**

  Sends a request to the extender to reply with message containing the current Elevator value.

  */
  void getElevator();

  /**

  Sends a request to the extender to reply with message containing the current Throttle value.

  */
  void getThrottle();

  /**

  Sends a request to the extender to reply with message containing the current Rudder value.

  */
  void getRudder();

  /**

  Sends a request to the extender to reply with message containing the current Flight Mode value.

  */
  void getFlightMode();

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void aileronCallback(void (*function)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void flightModeCallback(void (*function)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void throttleCallback(void (*function)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void rudderCallback(void (*function)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void elevatorCallback(void (*function)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron);

  /**

  Synchronous version of `getAileron`.

  */
  int16_t getAileronSync();

  /**

  Synchronous version of `getElevator`.

  */
  int16_t getElevatorSync();

  /**

  Synchronous version of `getThrottle`.

  */
  int16_t getThrottleSync();

  /**

  Synchronous version of `getRudder`.

  */
  int16_t getRudderSync();

  /**

  Synchronous version of `getFlightMode`.

  */
  int16_t getFlightModeSync();
};

#endif /* __ArduinoSDK__RC__ */

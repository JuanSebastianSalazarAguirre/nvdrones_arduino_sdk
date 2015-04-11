/*
 * AvantRC.h
 *
 *  Created on: Apr 10, 2015
 *      Author: amey
 */

#ifndef AVANTRC_H_
#define AVANTRC_H_

#include "Callback.h"
#include "SerialIO.h"

class AvantRC //handles sending values to the PWM/PPM port(s)
/**

 This class allows you to control your drone: set the throttle, rudder, elevator, and aileron channels on the drone.
 Essentially controlling up down left right ect.

*/
{
  private:
    //\cond
    SerialIO *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantRC();

    AvantRC(SerialIO *rcTservice, Callback *callback);
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
    void aileronCallback(void (*function)(byte));

    /**

    Execute funtion in a thread to not freez the drone.

    */
    void flightModeCallback(void (*function)(byte));

    /**

    Execute funtion in a thread to not freez the drone.

    */
    void throttleCallback(void (*function)(byte));

    /**

    Execute funtion in a thread to not freez the drone.

    */
    void rudderCallback(void (*function)(byte));

    /**

    Execute funtion in a thread to not freez the drone.

    */
    void elevatorCallback(void (*function)(byte));

    /**

    Execute funtion in a thread to not freez the drone.

    */
    void sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron);
};




#endif /* AVANTRC_H_ */

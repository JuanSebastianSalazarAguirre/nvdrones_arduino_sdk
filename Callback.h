/*
 * Callback.h
 *
 *  Created on: Apr 10, 2015
 *      Author: amey
 */

#ifndef CALLBACK_H_
#define CALLBACK_H_
#include <Arduino.h>

class Callback
//\endcond
{
  public:
    //SPI Callbacks
    void (*transfer)(byte);
    //I2C Callbacks
    void (*i2cRead)(byte);
    //GPS callbacks
    void (*longitude)(float);
    void (*latitude)(float);
    void (*altitude)(float);
    void (*speed)(float);
    void (*satellite)(byte);
    void (*orientation)(float);
    //RC Callbacks
    void (*flightMode)(byte);
    void (*elevator)(byte);
    void (*aileron)(byte);
    void (*throttle)(byte);
    void (*rudder)(byte);
    //pulseIn Callbacks
    void (*pulseIn1)(long);
    void (*pulseIn2)(long);
    void (*pulseIn3)(long);
    void (*pulseIn4)(long);
    void (*pulseIn5)(long);
    void (*pulseIn6)(long);
    void (*pulseIn7)(long);
    void (*pulseIn8)(long);
    void (*pulseIn9)(long);
    void (*pulseIn10)(long);
    //digital Read Callbacks
    void (*digitalRead1)(byte);
    void (*digitalRead2)(byte);
    void (*digitalRead3)(byte);
    void (*digitalRead4)(byte);
    void (*digitalRead5)(byte);
    void (*digitalRead6)(byte);
    void (*digitalRead7)(byte);
    void (*digitalRead8)(byte);
    void (*digitalRead9)(byte);
    void (*digitalRead10)(byte);
    //analog Read Callbacks
    void (*analogRead1)(byte);
    void (*analogRead2)(byte);
    void (*analogRead3)(byte);
    void (*analogRead4)(byte);
    //AutoPilot Callbacks
    void (*getWaypointLatitude)(float);
    void (*getWaypointLongitude)(float);
    void (*getWaypointAltitude)(float);
    void (*getWaypointOrientation)(float);
    void (*getElevatorKP)(float);
    void (*getElevatorKD)(float);
    void (*getElevatorKI)(float);
    void (*getThrottleKP)(float);
    void (*getThrottleKD)(float);
    void (*getThrottleKI)(float);
    void (*getAileronKP)(float);
    void (*getAileronKD)(float);
    void (*getAileronKI)(float);
    void (*getYawKP)(float);
    void (*getYawKD)(float);
    void (*getYawKI)(float);

};




#endif /* CALLBACK_H_ */

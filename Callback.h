#ifndef __ArduinoSDK__Callback__
#define __ArduinoSDK__Callback__

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
    void (*flightMode)(int16_t);
    void (*elevator)(int16_t);
    void (*aileron)(int16_t);
    void (*throttle)(int16_t);
    void (*rudder)(int16_t);
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
    void (*waypointLatitude)(float);
    void (*waypointLongitude)(float);
    void (*waypointAltitude)(float);
    void (*waypointOrientation)(float);
    void (*elevatorKP)(float);
    void (*elevatorKD)(float);
    void (*elevatorKI)(float);
    void (*throttleKP)(float);
    void (*throttleKD)(float);
    void (*throttleKI)(float);
    void (*aileronKP)(float);
    void (*aileronKD)(float);
    void (*aileronKI)(float);
    void (*yawKP)(float);
    void (*yawKD)(float);
    void (*yawKI)(float);

};

#endif /* defined __ArduinoSDK__Callback__ */
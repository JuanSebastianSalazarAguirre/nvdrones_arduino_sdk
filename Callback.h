
#ifndef NVCallbacks_h
#define NVCallbacks_h

#include <inttypes.h>

class Callback
{
public:
  Callback();

  // I2C
  void (*i2cRead)(uint8_t);
  void (*i2cAvailable)(int16_t);

  // GPS
  void (*longitude)(float);
  void (*latitude)(float);
  void (*altitude)(float);
  void (*speed)(float);
  void (*satellite)(int16_t);
  void (*yaw)(float);
  void (*pitchAngle)(float);
  void (*rollAngle)(float);

  // RC
  void (*flightMode)(int16_t);
  void (*elevator)(int16_t);
  void (*aileron)(int16_t);
  void (*throttle)(int16_t);
  void (*rudder)(int16_t);

  // pulseIn
  void (*pulseIn1)(uint32_t);
  void (*pulseIn2)(uint32_t);
  void (*pulseIn3)(uint32_t);
  void (*pulseIn4)(uint32_t);
  void (*pulseIn5)(uint32_t);
  void (*pulseIn6)(uint32_t);
  void (*pulseIn7)(uint32_t);
  void (*pulseIn8)(uint32_t);
  void (*pulseIn9)(uint32_t);
  void (*pulseIn10)(uint32_t);

  // digitalRead
  void (*digitalRead1)(int16_t);
  void (*digitalRead2)(int16_t);
  void (*digitalRead3)(int16_t);
  void (*digitalRead4)(int16_t);
  void (*digitalRead5)(int16_t);
  void (*digitalRead6)(int16_t);
  void (*digitalRead7)(int16_t);
  void (*digitalRead8)(int16_t);
  void (*digitalRead9)(int16_t);
  void (*digitalRead10)(int16_t);

  // analogRead
  void (*analogRead1)(int16_t);
  void (*analogRead2)(int16_t);
  void (*analogRead3)(int16_t);
  void (*analogRead4)(int16_t);

  // interrupt
  void (*interrupt0)(void);
  void (*interrupt1)(void);

  // vitals
  void (*heartbeatLost)(void);
  void (*heartbeatFound)(void);
  void (*voltage)(int16_t);
  void (*signalStrength)(int16_t);

  // errorHandler
  void (*errorHandler)(int16_t);
};

#endif // NVCallbacks_h

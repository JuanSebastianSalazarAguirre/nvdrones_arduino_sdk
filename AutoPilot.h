#ifndef __ArduinoSDK__AutoPilot__
#define __ArduinoSDK__AutoPilot__

#include "SerialIO.h"
#include "Callback.h"

class AutoPilot
{
private:
  SerialIO *serialIO;
  Callback *callbacks;

public:
  AutoPilot();
  AutoPilot(SerialIO *_serialIO, Callback *_callbacks);
  void gpsExecute();
  void compassExecute();
  void setYawError(float error);
  void setThrottleError(float error);
  void setElevatorError(float error);
  void setAileronError(float error);
  void setWaypointLatitude(float latitude);
  void setWaypointLongitude(float longitude);
  void setWaypointAltitude(float altitude);
  void setWaypointOrientation(float orientation);
  void setYawKP(float kp);
  void setYawKD(float kd);
  void setYawKI(float ki);
  void setThrottleKP(float kp);
  void setThrottleKD(float kd);
  void setThrottleKI(float ki);
  void setElevatorKP(float kp);
  void setElevatorKD(float kd);
  void setElevatorKI(float ki);
  void setAileronKP(float kp);
  void setAileronKD(float kd);
  void setAileronKI(float ki);
  void setYawMax(float max);
  void setThrottleMax(float max);
  void setElevatorMax(float max);
  void setAileronMax(float max);
  void getWaypointLatitude();
  void getWaypointLongitude();
  void getWaypointAltitude();
  void getWaypointOrientation();
  void getYawKP();
  void getYawKD();
  void getYawKI();
  void getThrottleKP();
  void getThrottleKD();
  void getThrottleKI();
  void getElevatorKP();
  void getElevatorKD();
  void getElevatorKI();
  void getAileronKP();
  void getAileronKD();
  void getAileronKI();
  void getYawMax();
  void getThrottleMax();
  void getElevatorMax();
  void getAileronMax();
  void waypointLatitudeCallback(void (*cb)(float));
  void waypointLongitudeCallback(void (*cb)(float));
  void waypointAltitudeCallback(void (*cb)(float));
  void waypointOrientationCallback(void (*cb)(float));
  void yawKPCallback(void (*cb)(float));
  void yawKDCallback(void (*cb)(float));
  void yawKICallback(void (*cb)(float));
  void throttleKPCallback(void (*cb)(float));
  void throttleKDCallback(void (*cb)(float));
  void throttleKICallback(void (*cb)(float));
  void elevatorKPCallback(void (*cb)(float));
  void elevatorKDCallback(void (*cb)(float));
  void elevatorKICallback(void (*cb)(float));
  void aileronKPCallback(void (*cb)(float));
  void aileronKDCallback(void (*cb)(float));
  void aileronKICallback(void (*cb)(float));
  void yawMaxCallback(void (*cb)(float));
  void throttleMaxCallback(void (*cb)(float));
  void elevatorMaxCallback(void (*cb)(float));
  void aileronMaxCallback(void (*cb)(float));
};

#endif /* defined __ArduinoSDK__AutoPilot__ */

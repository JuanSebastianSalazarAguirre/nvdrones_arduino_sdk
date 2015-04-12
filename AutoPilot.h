#ifndef __ArduinoSDK__AutoPilot__
#define __ArduinoSDK__AutoPilot__

#include "SerialIO.h"
#include "Callback.h"

class AutoPilot
{
private:
  SerialIO *service;
  Callback *myCallback;

public:
  AutoPilot();
  AutoPilot(SerialIO *rcTservice, Callback *callback);
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
  void setWaypointLatitudeCallback(void (*cb)(float));
  void setWaypointLongitudeCallback(void (*cb)(float));
  void setWaypointAltitudeCallback(void (*cb)(float));
  void setWaypointOrientationCallback(void (*cb)(float));
  void setYawKPCallback(void (*cb)(float));
  void setYawKDCallback(void (*cb)(float));
  void setYawKICallback(void (*cb)(float));
  void setThrottleKPCallback(void (*cb)(float));
  void setThrottleKDCallback(void (*cb)(float));
  void setThrottleKICallback(void (*cb)(float));
  void setElevatorKPCallback(void (*cb)(float));
  void setElevatorKDCallback(void (*cb)(float));
  void setElevatorKICallback(void (*cb)(float));
  void setAileronKPCallback(void (*cb)(float));
  void setAileronKDCallback(void (*cb)(float));
  void setAileronKICallback(void (*cb)(float));
};

#endif /* defined __ArduinoSDK__AutoPilot__ */
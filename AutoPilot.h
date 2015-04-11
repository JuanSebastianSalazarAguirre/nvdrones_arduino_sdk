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
    void getWaypointLatitude(void (*function)(float));
    void getWaypointLongitude(void (*function)(float));
    void getWaypointAltitude(void (*function)(float));
    void getWaypointOrientation(void (*function)(float));
    void getYawKP(void (*function)(float));
    void getYawKD(void (*function)(float));
    void getYawKI(void (*function)(float));
    void getThrottleKP(void (*function)(float));
    void getThrottleKD(void (*function)(float));
    void getThrottleKI(void (*function)(float));
    void getElevatorKP(void (*function)(float));
    void getElevatorKD(void (*function)(float));
    void getElevatorKI(void (*function)(float));
    void getAileronKP(void (*function)(float));
    void getAileronKD(void (*function)(float));
    void getAileronKI(void (*function)(float));
};

#endif
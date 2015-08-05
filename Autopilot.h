#ifndef NV_AUTOPILOT
#define NV_AUTOPILOT

#include "Callback.h"
#include "SerialIO.h"
#include "IncomingPacketReader.h"

class Autopilot {
private:
  SerialIO *_serialIO;
  Callback *_callbacks;
  IncomingPacketReader *_incomingPacketReader;
public:

  //constructors
  Autopilot();
  Autopilot(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader);

  //setters
  void setModeTakeover();
  void setModeIndoorHover();
  void setModeOutdoorHover();
  void setAutopilotMode(int);
  
  void setAltitudeKp(float);
  void setAltitudeKi(float);
  void setAltitudeBase(float);
  void setAltitudeReference(float);
  void setAltitudeTolerance(float);

  void setXPositionKp(float);
  void setXPositionKi(float);
  void setXPositionKd(float);
  void setXPositionReference(float);
  void setXPositionTolerance(float);

  void setYPositionKp(float);
  void setYPositionKi(float);
  void setYPositionKd(float);
  void setYPositionReference(float);
  void setYPositionTolerance(float);

  //callbacks
  void indoorHoverCallback(void (*function)(float));
  void outdoorHoverCallback(void (*function)(float));
  void takeoverCallback(void(*function)(float));

  void sonarAltitudeCallback(void (*function)(float));
  void sonarXPositionCallback(void (*function)(float));
  void sonarYPositionCallback(void (*function)(float));

  void altitudeKpCallback(void(*function)(float));
  void altitudeKiCallback(void(*function)(float));
  void altitudeBaseCallback(void(*function)(float));
  void altitudeReferenceCallback(void(*function)(float));
  void altitudeToleranceCallback(void(*function)(float));

  void XPositionKpCallback(void(*function)(float));
  void XPositionKiCallback(void(*function)(float));
  void XPositionKdCallback(void(*function)(float));
  void XPositionReferenceCallback(void(*function)(float));
  void XPositionToleranceCallback(void(*function)(float));

  void YPositionKpCallback(void(*function)(float));
  void YPositionKiCallback(void(*function)(float));
  void YPositionKdCallback(void(*function)(float));
  void YPositionReferenceCallback(void(*function)(float));
  void YPositionToleranceCallback(void(*function)(float));

  //getters
  void getSonarAltitude();
  void getSonarXPosition();
  void getSonarYPosition();

  void getAltitudeKp();
  void getAltitudeKi();
  void getAltitudeBase();
  void getAltitudeReference();
  void getAltitudeTolerance();

  void getXPositionKp();
  void getXPositionKi();
  void getXPositionKd();
  void getXPositionReference();
  void getXPositionTolerance();
  
  void getYPositionKp();
  void getYPositionKi();
  void getYPositionKd();
  void getYPositionReference();
  void getYPositionTolerance();

  
  //sync getters
  float getAltitudeKpSync();
  float getAltitudeKiSync();
  float getAltitudeBaseSync();
  float getAltitudeReferenceSync();
  float getAltitudeToleranceSync();

  float getXPositionKpSync();
  float getXPositionKiSync();
  float getXPositionKdSync();
  float getXPositionReferenceSync();
  float getXPositionToleranceSync();

  float getYPositionKpSync();
  float getYPositionKiSync();
  float getYPositionKdSync();
  float getYPositionReferenceSync();
  float getYPositionToleranceSync();

  float getSonarAltitudeSync();
  float getSonarXPositionSync();
  float getSonarYPositionSync();

};

#endif //NV_AUTOPILOT
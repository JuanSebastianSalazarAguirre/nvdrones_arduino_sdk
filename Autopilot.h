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
  
  void setAltitudeKp(int);
  void setAltitudeKi(int);
  void setAltitudeBase(int);
  void setAltitudeReference(float);
  void setAltitudeTolerance(float);

  void setXPositionKp(int);
  void setXPositionKi(int);
  void setXPositionKd(int);
  void setXPositionReference(float);
  void setXPositionTolerance(float);

  void setYPositionKp(int);
  void setYPositionKi(int);
  void setYPositionKd(int);
  void setYPositionReference(float);
  void setYPositionTolerance(float);

  //callbacks
  void indoorHoverCallback(void (*function)(float));
  void outdoorHoverCallback(void (*function)(float));
  void takeoverCallback(void(*function)(float));

  void sonarAltitudeCallback(void (*function)(float));
  void sonarXPositionCallback(void (*function)(float));
  void sonarYPositionCallback(void (*function)(float));

  void altitudeKpCallback(void(*function)(int16_t));
  void altitudeKiCallback(void(*function)(int16_t));
  void altitudeBaseCallback(void(*function)(int16_t));
  void altitudeReferenceCallback(void(*function)(float));
  void altitudeToleranceCallback(void(*function)(float));

  void XPositionKpCallback(void(*function)(int16_t));
  void XPositionKiCallback(void(*function)(int16_t));
  void XPositionKdCallback(void(*function)(int16_t));
  void XPositionReferenceCallback(void(*function)(float));
  void XPositionToleranceCallback(void(*function)(float));

  void YPositionKpCallback(void(*function)(int16_t));
  void YPositionKiCallback(void(*function)(int16_t));
  void YPositionKdCallback(void(*function)(int16_t));
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
  int16_t getAltitudeKpSync();
  int16_t getAltitudeKiSync();
  int16_t getAltitudeBaseSync();
  float getAltitudeReferenceSync();
  float getAltitudeToleranceSync();

  int16_t getXPositionKpSync();
  int16_t getXPositionKiSync();
  int16_t getXPositionKdSync();
  float getXPositionReferenceSync();
  float getXPositionToleranceSync();

  int16_t getYPositionKpSync();
  int16_t getYPositionKiSync();
  int16_t getYPositionKdSync();
  float getYPositionReferenceSync();
  float getYPositionToleranceSync();

  int16_t getSonarAltitudeSync();
  int16_t getSonarXPositionSync();
  int16_t getSonarYPositionSync();

};

#endif //NV_AUTOPILOT
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
  Autopilot();
  Autopilot(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader);

  //chooses which autopilot feature to enable
  void setModeTakeover();
  void setModeIndoorHover();
  void setModeOutdoorHover();
  void setAutopilotMode(int);

  //callbacks
  void indoorHoverCallback(void (*function)(float));
  void outdoorHoverCallback(void (*function)(float));
  void takeoverCallback(void(*function)(float));
  void sonarAltitudeCallback(void (*function)(float));
  void sonarXPositionCallback(void (*function)(float));
  void sonarYPositionCallback(void (*function)(float));
  void getSonarAltitude();
  void getSonarXPosition();
  void getSonarYPosition();
  int16_t getSonarAltitudeSync();
  int16_t getSonarPositionXSync();
  int16_t getSonarPositionYSync();

};

#endif //NV_AUTOPILOT